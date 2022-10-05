// Copyright 2022 iRobot Corporation. All Rights Reserved

#include "rclcpp/executors/events_executor/events_executor_entities_collector.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/executors/events_executor/events_executor.hpp"

using rclcpp::executors::ExecutorEvent;
using rclcpp::executors::ExecutorEventType;
using rclcpp::executors::EventsExecutorEntitiesCollector;

EventsExecutorEntitiesCollector::EventsExecutorEntitiesCollector(
  rclcpp::executors::EventsExecutor * executor)
{
  if (executor == nullptr) {
    throw std::runtime_error("Received nullptr executor in EventsExecutorEntitiesCollector.");
  }

  associated_executor_ = executor;
  timers_manager_ = associated_executor_->timers_manager_;
}

EventsExecutorEntitiesCollector::~EventsExecutorEntitiesCollector()
{
  std::lock_guard<std::recursive_mutex> lock(reentrant_mutex_);

  // Disassociate all callback groups and thus nodes.
  for (const auto & pair : weak_groups_associated_with_executor_to_nodes_) {
    auto group = pair.first.lock();
    if (group) {
      std::atomic_bool & has_executor = group->get_associated_with_executor_atomic();
      has_executor.store(false);
      callback_group_removed_impl(group);
    }
  }
  for (const auto & pair : weak_groups_to_nodes_associated_with_executor_) {
    auto group = pair.first.lock();
    if (group) {
      std::atomic_bool & has_executor = group->get_associated_with_executor_atomic();
      has_executor.store(false);
      callback_group_removed_impl(group);
    }
  }
  // Disassociate all nodes
  for (const auto & weak_node : weak_nodes_) {
    auto node = weak_node.lock();
    if (node) {
      std::atomic_bool & has_executor = node->get_associated_with_executor_atomic();
      has_executor.store(false);
      node_removed_impl(node);
    }
  }

  weak_clients_map_.clear();
  weak_services_map_.clear();
  weak_waitables_map_.clear();
  weak_subscriptions_map_.clear();
  weak_nodes_to_guard_conditions_.clear();
}

void
EventsExecutorEntitiesCollector::init()
{
  std::lock_guard<std::recursive_mutex> lock(reentrant_mutex_);
  // Add the EventsExecutorEntitiesCollector shared_ptr to waitables map
  weak_waitables_map_.emplace(this, this->shared_from_this());
}

void
EventsExecutorEntitiesCollector::execute(std::shared_ptr<void> & data)
{
  // This function is called when the associated executor is notified that something changed.
  // We do not know if an entity has been added or removed so we have to rebuild everything.
  (void)data;

  std::lock_guard<std::recursive_mutex> lock(reentrant_mutex_);

  timers_manager_->clear();

  // If a registered node has a new callback group, register the group.
  add_callback_groups_from_nodes_associated_to_executor();

  // For all groups registered in the executor, set their event callbacks.
  set_entities_event_callbacks_from_map(weak_groups_associated_with_executor_to_nodes_);
  set_entities_event_callbacks_from_map(weak_groups_to_nodes_associated_with_executor_);
}

void
EventsExecutorEntitiesCollector::add_to_wait_set(rcl_wait_set_t * wait_set)
{
  (void)wait_set;
}

bool
EventsExecutorEntitiesCollector::is_ready(rcl_wait_set_t * p_wait_set)
{
  (void)p_wait_set;
  return false;
}

void
EventsExecutorEntitiesCollector::callback_group_added_impl(
  rclcpp::CallbackGroup::SharedPtr group)
{
  std::lock_guard<std::recursive_mutex> lock(reentrant_mutex_);
  // For all entities in the callback group, set their event callback
  set_callback_group_entities_callbacks(group);
}

void
EventsExecutorEntitiesCollector::node_added_impl(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node)
{
  std::lock_guard<std::recursive_mutex> lock(reentrant_mutex_);

  auto notify_guard_condition = &(node->get_notify_guard_condition());
  // Set an event callback for the node's notify guard condition, so if new entities are added
  // or removed to this node we will receive an event.
  set_guard_condition_callback(notify_guard_condition);

  // Store node's notify guard condition
  weak_nodes_to_guard_conditions_[node] = notify_guard_condition;
}

void
EventsExecutorEntitiesCollector::callback_group_removed_impl(
  rclcpp::CallbackGroup::SharedPtr group)
{
  std::lock_guard<std::recursive_mutex> lock(reentrant_mutex_);
  // For all the entities in the group, unset their callbacks
  unset_callback_group_entities_callbacks(group);
}

void
EventsExecutorEntitiesCollector::node_removed_impl(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node)
{
  std::lock_guard<std::recursive_mutex> lock(reentrant_mutex_);
  // Node doesn't have more callback groups associated to the executor.
  // Unset the event callback for the node's notify guard condition, to stop
  // receiving events if entities are added or removed to this node.
  unset_guard_condition_callback(&(node->get_notify_guard_condition()));

  // Remove guard condition from list
  rclcpp::node_interfaces::NodeBaseInterface::WeakPtr weak_node_ptr(node);
  weak_nodes_to_guard_conditions_.erase(weak_node_ptr);
}

void
EventsExecutorEntitiesCollector::set_entities_event_callbacks_from_map(
  const WeakCallbackGroupsToNodesMap & weak_groups_to_nodes)
{
  for (const auto & pair : weak_groups_to_nodes) {
    auto group = pair.first.lock();
    auto node = pair.second.lock();
    if (!node || !group || !group->can_be_taken_from().load()) {
      continue;
    }
    set_callback_group_entities_callbacks(group);
  }
}

void
EventsExecutorEntitiesCollector::set_callback_group_entities_callbacks(
  rclcpp::CallbackGroup::SharedPtr group)
{
  // Timers are handled by the timers manager
  group->find_timer_ptrs_if(
    [this](const rclcpp::TimerBase::SharedPtr & timer) {
      if (timer) {
        timers_manager_->add_timer(timer);
      }
      return false;
    });

  // Set callbacks for all other entity types
  group->find_subscription_ptrs_if(
    [this](const rclcpp::SubscriptionBase::SharedPtr & subscription) {
      assert(this != nullptr);
      if (subscription) {
        weak_subscriptions_map_.emplace(subscription.get(), subscription);

        subscription->set_on_new_message_callback(
          create_entity_callback(subscription.get(), ExecutorEventType::SUBSCRIPTION_EVENT));
      }
      return false;
    });
  group->find_service_ptrs_if(
    [this](const rclcpp::ServiceBase::SharedPtr & service) {
      if (service) {
        weak_services_map_.emplace(service.get(), service);

        service->set_on_new_request_callback(
          create_entity_callback(service.get(), ExecutorEventType::SERVICE_EVENT));
      }
      return false;
    });
  group->find_client_ptrs_if(
    [this](const rclcpp::ClientBase::SharedPtr & client) {
      if (client) {
        weak_clients_map_.emplace(client.get(), client);

        client->set_on_new_response_callback(
          create_entity_callback(client.get(), ExecutorEventType::CLIENT_EVENT));
      }
      return false;
    });
  group->find_waitable_ptrs_if(
    [this](const rclcpp::Waitable::SharedPtr & waitable) {
      if (waitable) {
        weak_waitables_map_.emplace(waitable.get(), waitable);

        waitable->set_on_ready_callback(
          create_waitable_callback(waitable.get()));
      }
      return false;
    });
}

void
EventsExecutorEntitiesCollector::unset_callback_group_entities_callbacks(
  rclcpp::CallbackGroup::SharedPtr group)
{
  // Timers are handled by the timers manager
  group->find_timer_ptrs_if(
    [this](const rclcpp::TimerBase::SharedPtr & timer) {
      if (timer) {
        timers_manager_->remove_timer(timer);
      }
      return false;
    });

  // Unset callbacks for all other entity types
  group->find_subscription_ptrs_if(
    [this](const rclcpp::SubscriptionBase::SharedPtr & subscription) {
      if (subscription) {
        subscription->clear_on_new_message_callback();
        weak_subscriptions_map_.erase(subscription.get());
      }
      return false;
    });
  group->find_service_ptrs_if(
    [this](const rclcpp::ServiceBase::SharedPtr & service) {
      if (service) {
        service->clear_on_new_request_callback();
        weak_services_map_.erase(service.get());
      }
      return false;
    });
  group->find_client_ptrs_if(
    [this](const rclcpp::ClientBase::SharedPtr & client) {
      if (client) {
        client->clear_on_new_response_callback();
        weak_clients_map_.erase(client.get());
      }
      return false;
    });
  group->find_waitable_ptrs_if(
    [this](const rclcpp::Waitable::SharedPtr & waitable) {
      if (waitable) {
        waitable->clear_on_ready_callback();
        weak_waitables_map_.erase(waitable.get());
      }
      return false;
    });
}

void
EventsExecutorEntitiesCollector::set_guard_condition_callback(
  rclcpp::GuardCondition * guard_condition)
{
  auto gc_callback = [this](size_t num_events) {
      // Override num events (we don't care more than a single event)
      num_events = 1;
      int gc_id = -1;
      ExecutorEvent event = {this, gc_id, ExecutorEventType::WAITABLE_EVENT, num_events};
      associated_executor_->events_queue_->enqueue(event);
    };

  guard_condition->set_on_trigger_callback(gc_callback);
}

void
EventsExecutorEntitiesCollector::unset_guard_condition_callback(
  rclcpp::GuardCondition * guard_condition)
{
  guard_condition->set_on_trigger_callback(nullptr);
}

rclcpp::SubscriptionBase::SharedPtr
EventsExecutorEntitiesCollector::get_subscription(const void * subscription_id)
{
  std::lock_guard<std::recursive_mutex> lock(reentrant_mutex_);

  auto it = weak_subscriptions_map_.find(subscription_id);

  if (it != weak_subscriptions_map_.end()) {
    auto subscription_weak_ptr = it->second;
    auto subscription_shared_ptr = subscription_weak_ptr.lock();

    if (subscription_shared_ptr) {
      return subscription_shared_ptr;
    }

    // The subscription expired, remove from map
    weak_subscriptions_map_.erase(it);
  }
  return nullptr;
}

rclcpp::ClientBase::SharedPtr
EventsExecutorEntitiesCollector::get_client(const void * client_id)
{
  std::lock_guard<std::recursive_mutex> lock(reentrant_mutex_);

  auto it = weak_clients_map_.find(client_id);

  if (it != weak_clients_map_.end()) {
    auto client_weak_ptr = it->second;
    auto client_shared_ptr = client_weak_ptr.lock();

    if (client_shared_ptr) {
      return client_shared_ptr;
    }

    // The client expired, remove from map
    weak_clients_map_.erase(it);
  }
  return nullptr;
}

rclcpp::ServiceBase::SharedPtr
EventsExecutorEntitiesCollector::get_service(const void * service_id)
{
  std::lock_guard<std::recursive_mutex> lock(reentrant_mutex_);

  auto it = weak_services_map_.find(service_id);

  if (it != weak_services_map_.end()) {
    auto service_weak_ptr = it->second;
    auto service_shared_ptr = service_weak_ptr.lock();

    if (service_shared_ptr) {
      return service_shared_ptr;
    }

    // The service expired, remove from map
    weak_services_map_.erase(it);
  }
  return nullptr;
}

rclcpp::Waitable::SharedPtr
EventsExecutorEntitiesCollector::get_waitable(const void * waitable_id)
{
  std::lock_guard<std::recursive_mutex> lock(reentrant_mutex_);

  auto it = weak_waitables_map_.find(waitable_id);

  if (it != weak_waitables_map_.end()) {
    auto waitable_weak_ptr = it->second;
    auto waitable_shared_ptr = waitable_weak_ptr.lock();

    if (waitable_shared_ptr) {
      return waitable_shared_ptr;
    }

    // The waitable expired, remove from map
    weak_waitables_map_.erase(it);
  }
  return nullptr;
}

void
EventsExecutorEntitiesCollector::add_waitable(rclcpp::Waitable::SharedPtr waitable)
{
  std::lock_guard<std::recursive_mutex> lock(reentrant_mutex_);

  weak_waitables_map_.emplace(waitable.get(), waitable);

  waitable->set_on_ready_callback(
    create_waitable_callback(waitable.get()));
}

std::function<void(size_t)>
EventsExecutorEntitiesCollector::create_entity_callback(
  void * exec_entity_id, ExecutorEventType event_type)
{
  assert(this != nullptr);
  std::function<void(size_t)> callback = [this, exec_entity_id, event_type](size_t num_events) {
      assert(this != nullptr);
      ExecutorEvent event = {exec_entity_id, -1, event_type, num_events};
      associated_executor_->events_queue_->enqueue(event);
    };
  return callback;
}

std::function<void(size_t, int)>
EventsExecutorEntitiesCollector::create_waitable_callback(void * exec_entity_id)
{
  std::function<void(size_t, int)> callback = [this, exec_entity_id](size_t num_events, int gen_entity_id) {
      ExecutorEvent event =
      {exec_entity_id, gen_entity_id, ExecutorEventType::WAITABLE_EVENT, num_events};
      associated_executor_->events_queue_->enqueue(event);
    };
  return callback;
}
