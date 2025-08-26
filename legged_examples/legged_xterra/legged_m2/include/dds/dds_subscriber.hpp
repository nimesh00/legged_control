#ifndef __DDS_SUBSCRIBER_HPP__
#define __DDS_SUBSCRIBER_HPP__

#include <iostream>
#include <functional>
#include <dds/dds.hpp>
#include <thread>
// using namespace dds::sub;
// using namespace dds::domain;
// using namespace dds::topic;

template<typename MSG_TYPE>
class DDSSubscriber {
public:
    // Define a callback type for user-provided callbacks
    
    using UserCallback = std::function<void(const MSG_TYPE&)>;

private:
    dds::domain::DomainParticipant participant;
    dds::topic::Topic<MSG_TYPE> topic;
    dds::sub::Subscriber subscriber;
    dds::sub::DataReader<MSG_TYPE> reader;
    MSG_TYPE latestMessage;
    UserCallback userCallback;

    // Internal Listener class
    class CustomListener : public dds::sub::NoOpDataReaderListener<MSG_TYPE> {
    private:
        DDSSubscriber& parent;

    public:
        CustomListener(DDSSubscriber& parent) : parent(parent) {}

        void on_data_available(dds::sub::DataReader<MSG_TYPE>& reader) override {
            bool message_received = false;
            // Take all available data
            auto samples = reader.take();
            if (samples.length() > 0) {
                for (const auto& sample : samples) {
                    if (message_received) {
                        break;
                    }
                    const auto& info = sample.info();
                    if (info.valid()) {
                        message_received = true;
                        parent.latestMessage = sample->data(); // Update the latest message
                        // std::cout << "Sample data stored\n";
                        // If a user callback is provided, execute it
                        if (parent.userCallback) {
                            parent.userCallback(parent.latestMessage);
                        }
                    }
                }
            }
        }
    };

public:
    // Constructor
    DDSSubscriber(const std::string& topicName, UserCallback callback = nullptr, const int id = 0)
        : participant(id),
        topic(participant, topicName),
        subscriber(participant),
        userCallback(callback),
        reader(subscriber, topic, dds::sub::qos::DataReaderQos(), new CustomListener(*this), dds::core::status::StatusMask::data_available()) {}

    // Method to get the latest message
    MSG_TYPE getLatestMessage() const {
        return latestMessage;
    }
};
#endif