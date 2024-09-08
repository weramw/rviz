/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <sstream>

#include <OgreSceneNode.h>

#include <rviz/default_plugin/markers/marker_base.h>
#include <rviz/default_plugin/marker_utils.h>
#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/ogre_helpers/arrow.h>
#include <rviz/ogre_helpers/billboard_line.h>
#include <rviz/ogre_helpers/shape.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/selection/selection_manager.h>
#include <rviz/windows_compat.h>

#include <rviz/default_plugin/marker_display.h>


namespace rviz
{
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

MarkerDisplay::MarkerDisplay() : Display(), tf_filter_(nullptr)
{
    marker_topic_property_ = new RosTopicProperty(
        "Marker Topic", "visualization_marker",
        QString::fromStdString(ros::message_traits::datatype<visualization_msgs::Marker>()),
        "visualization_msgs::Marker topic to subscribe to.  <topic>_array will also"
        " automatically be subscribed with type visualization_msgs::MarkerArray.",
        this, &MarkerDisplay::updateTopic);

    queue_size_property_ =
        new IntProperty("Queue Size", 100,
                        "Advanced: set the size of the incoming Marker message queue.  Increasing this is"
                        " useful if your incoming TF data is delayed significantly from your Marker data, "
                        "but it can greatly increase memory usage if the messages are big.",
                        this, &MarkerDisplay::updateQueueSize);
    queue_size_property_->setMin(0);

    namespaces_category_ = new Property("Namespaces", QVariant(), "", this);
}

void MarkerDisplay::onInitialize()
{
    tf_filter_ =
        new tf2_ros::MessageFilter<visualization_msgs::Marker>(*context_->getTF2BufferPtr(),
                                                               fixed_frame_.toStdString(),
                                                               queue_size_property_->getInt(), update_nh_);

    tf_filter_->connectInput(sub_);
    tf_filter_->registerCallback(
        boost::bind(&MarkerDisplay::incomingMarker, this, boost::placeholders::_1));
    tf_filter_->registerFailureCallback(
        boost::bind(&MarkerDisplay::failedMarker, this, boost::placeholders::_1, boost::placeholders::_2));

    namespace_config_enabled_state_.clear();
}

MarkerDisplay::~MarkerDisplay()
{
    if (initialized())
    {
        MarkerDisplay::unsubscribe();

        clearMarkers();

        delete tf_filter_;
    }
}

void MarkerDisplay::load(const Config& config)
{
    Display::load(config);

    Config c = config.mapGetChild("Namespaces");
    for (Config::MapIterator iter = c.mapIterator(); iter.isValid(); iter.advance())
    {
        QString key = iter.currentKey();
        const Config& child = iter.currentChild();
        namespace_config_enabled_state_[key] = child.getValue().toBool();
    }
}

void MarkerDisplay::clearMarkers()
{
    namespaces_.clear();
    if (tf_filter_) // also clear messages in pipeline
        tf_filter_->clear();
}

void MarkerDisplay::onEnable()
{
    subscribe();
    scene_node_->setVisible(true);
}

void MarkerDisplay::onDisable()
{
    unsubscribe();
    //reset(); // do NOT clear information here!
    scene_node_->setVisible(false); // ONLY disable rendering!
}

void MarkerDisplay::updateQueueSize()
{
    tf_filter_->setQueueSize((uint32_t)queue_size_property_->getInt());
    subscribe();
}

void MarkerDisplay::updateTopic()
{
    onDisable();
    onEnable();
}

void MarkerDisplay::subscribe()
{
    if (!isEnabled())
    {
        return;
    }

    std::string marker_topic = marker_topic_property_->getTopicStd();
    if (marker_topic.empty()){
        return;
    }
    
    array_sub_.shutdown();
    sub_.unsubscribe();

    try
    {
        sub_.subscribe(update_nh_, marker_topic, queue_size_property_->getInt());
        array_sub_ = update_nh_.subscribe(marker_topic + "_array", queue_size_property_->getInt(),
                                          &MarkerDisplay::incomingMarkerArray, this);
        setStatus(StatusProperty::Ok, "Topic", "OK");
    }
    catch (ros::Exception& e)
    {
        setStatus(StatusProperty::Error, "Topic", QString("Error subscribing: ") + e.what());
    }
    
}

void MarkerDisplay::unsubscribe()
{
    sub_.unsubscribe();
    array_sub_.shutdown();
}

inline void MarkerDisplay::deleteMarker(const MarkerID& id)
{
    M_Namespace::iterator ns_it = namespaces_.find(QString::fromStdString(id.first));
    if(ns_it == namespaces_.end()){
        return;
    }
    ns_it.value()->removeMarker(id.second);
    if(ns_it.value()->markers().empty()){
        namespaces_.erase(ns_it);
    }
}

void MarkerDisplay::deleteMarkersInNamespace(const std::string& ns)
{
    M_Namespace::iterator ns_it = namespaces_.find(QString::fromStdString(ns));
    if(ns_it == namespaces_.end()){
        return;
    }
    namespaces_.erase(ns_it);
}

void MarkerDisplay::deleteAllMarkers()
{
    namespaces_.clear();
}

void MarkerDisplay::setMarkerStatus(const MarkerID& id, StatusLevel level, const std::string& text)
{
    std::stringstream ss;
    ss << id.first << "/" << id.second;
    std::string marker_name = ss.str();
    setStatusStd(level, marker_name, text);
}

void MarkerDisplay::deleteMarkerStatus(const MarkerID& id)
{
    std::stringstream ss;
    ss << id.first << "/" << id.second;
    std::string marker_name = ss.str();
    deleteStatusStd(marker_name);
}

void MarkerDisplay::incomingMarkerArray(const visualization_msgs::MarkerArray::ConstPtr& array)
{
    checkMarkerArrayMsg(*array, this);
    for (const visualization_msgs::Marker& marker : array->markers)
    {
        tf_filter_->add(visualization_msgs::Marker::Ptr(new visualization_msgs::Marker(marker)));
    }
}

void MarkerDisplay::incomingMarker(const visualization_msgs::Marker::ConstPtr& marker)
{
    boost::mutex::scoped_lock lock(queue_mutex_);
    message_queue_.push_back(marker);
}

void MarkerDisplay::failedMarker(const ros::MessageEvent<visualization_msgs::Marker>& marker_evt,
                                 tf2_ros::FilterFailureReason reason)
{
    const visualization_msgs::Marker::ConstPtr& marker = marker_evt.getConstMessage();
    if (marker->action == visualization_msgs::Marker::DELETE ||
        marker->action == visualization_msgs::Marker::DELETEALL)
    {
        return this->processMessage(marker);
    }
    const std::string& authority = marker_evt.getPublisherName();
    std::string error = context_->getFrameManager()->discoverFailureReason(
        marker->header.frame_id, marker->header.stamp, authority, reason);

    setMarkerStatus(MarkerID(marker->ns, marker->id), StatusProperty::Error, error);
}

void MarkerDisplay::processMessage(const visualization_msgs::Marker::ConstPtr& message)
{
    switch (message->action)
    {
    case visualization_msgs::Marker::ADD:
        if (checkMarkerMsg(*message, this))
            processAdd(message);
        break;

    case visualization_msgs::Marker::DELETE:
        processDelete(message);
        break;

    case visualization_msgs::Marker::DELETEALL:
        deleteAllMarkers();
        break;

    default:
        ROS_ERROR("Unknown marker action: %d\n", message->action);
    }
}

void MarkerDisplay::processAdd(const visualization_msgs::Marker::ConstPtr& message)
{
    QString namespace_name = QString::fromStdString(message->ns);
    M_Namespace::iterator ns_it = namespaces_.find(namespace_name);
    if (ns_it == namespaces_.end())
    {
        ns_it = namespaces_.insert(namespace_name, boost::make_shared<MarkerNamespace>(
                                       namespace_name, namespaces_category_, this));

        // Adding a new namespace, determine if it's configured to be disabled
        if (namespace_config_enabled_state_.count(namespace_name) > 0 &&
            !namespace_config_enabled_state_[namespace_name])
        {
            ns_it.value()->setValue(false); // Disable the namespace
        }
    }

    bool create = true;
    MarkerBasePtr marker(createMarker(message->type, this, context_, scene_node_));
    ns_it.value()->addMarker(marker);
    context_->queueRender();
}

void MarkerDisplay::processDelete(const visualization_msgs::Marker::ConstPtr& message)
{
    deleteMarker(MarkerID(message->ns, message->id));
    context_->queueRender();
}

void MarkerDisplay::update(float /*wall_dt*/, float /*ros_dt*/)
{
    V_MarkerMessage local_queue;

    {
        boost::mutex::scoped_lock lock(queue_mutex_);

        local_queue.swap(message_queue_);
    }

    if (!local_queue.empty())
    {
        V_MarkerMessage::iterator message_it = local_queue.begin();
        V_MarkerMessage::iterator message_end = local_queue.end();
        for (; message_it != message_end; ++message_it)
        {
            visualization_msgs::Marker::ConstPtr& marker = *message_it;

            processMessage(marker);
        }
    }

    std::vector<QString> empty_namespaces;
    for(M_Namespace::iterator ns_it = namespaces_.begin(); ns_it != namespaces_.end(); ++ns_it){
        ns_it.value()->update();
        if(ns_it.value()->markers().empty()){
            empty_namespaces.emplace_back(ns_it.key());
        }
    }
    for(int i = 0; i < empty_namespaces.size(); ++i){
        namespaces_.remove(empty_namespaces[i]);
    }
}

void MarkerDisplay::fixedFrameChanged()
{
    tf_filter_->setTargetFrame(fixed_frame_.toStdString());

    clearMarkers();
}

void MarkerDisplay::reset()
{
    Display::reset();
    clearMarkers();
}

void MarkerDisplay::setTopic(const QString& topic, const QString& /*datatype*/)
{
    marker_topic_property_->setString(topic);
}

/////////////////////////////////////////////////////////////////////////////////
// MarkerNamespace

MarkerNamespace::MarkerNamespace(const QString& name, Property* parent_property, MarkerDisplay* owner)
    : BoolProperty(name, true, "Enable/disable all markers in this namespace.", parent_property)
    , owner_(owner)
{
    // Can't do this connect in chained constructor above because at
    // that point it doesn't really know that "this" is a
    // MarkerNamespace*, so the signal doesn't get connected.
    connect(this, &Property::changed, this, &MarkerNamespace::onEnableChanged);
}

void MarkerNamespace::onEnableChanged()
{
    bool is_enabled = isEnabled();
    // use Ogre logic to set markers visible/not visible
    for(int i = 0; i < markers_.size(); ++i){
        markers_[i]->setVisible(is_enabled);
    }

    // Update the configuration that stores the enabled state of all markers
    owner_->namespace_config_enabled_state_[getName()] = is_enabled;
}

bool MarkerNamespace::isInNamespace(const MarkerBasePtr& marker)
{
    return getName() == QString::fromStdString(marker->getID().first);
}

bool MarkerNamespace::addMarker(MarkerBasePtr marker)
{
    if(!isInNamespace(marker)){
        return false;
    }
    markers_[marker->getID().second] = marker; // this also overwrites marker of the same id
    return true;
}

bool MarkerNamespace::removeMarker(int32_t id)
{
    const std::unordered_map<int32_t, MarkerBasePtr>::iterator marker_it = markers_.find(id);
    if(marker_it == markers_.end()){
        return false;
    }
    markers_.erase(marker_it);
    return true;
}

void MarkerNamespace::update(){
    std::vector<int32_t> expired_markers;
    for(M_Marker::iterator m_it = markers_.begin(); m_it != markers_.end(); ++m_it){
        if (m_it->second->expired()){
            expired_markers.emplace_back(m_it->first);
            continue;
        }
        m_it->second->updateFrameLocked();
    }

    for(int i=0; i<expired_markers.size(); ++i){
        removeMarker(expired_markers[i]);
    }
}


} // namespace rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz::MarkerDisplay, rviz::Display)
