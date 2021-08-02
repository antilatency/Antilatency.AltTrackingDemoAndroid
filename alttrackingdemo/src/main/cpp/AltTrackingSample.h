#ifndef ANTILATENCY_ALTTRACKINGDEMOANDROID_ALTTRACKINGSAMPLE_H
#define ANTILATENCY_ALTTRACKINGDEMOANDROID_ALTTRACKINGSAMPLE_H

#include <string>
#include <jni.h>
#include <thread>
#include <mutex>

#include <Antilatency.Api.h>

extern "C"{
    JNIEXPORT void JNICALL
    Java_com_antilatency_alttrackingdemo_MainActivity_Init(JNIEnv* env, jobject instance);

    JNIEXPORT jstring JNICALL
    Java_com_antilatency_alttrackingdemo_MainActivity_GetOutput(JNIEnv* env, jobject instance);
};

class AltTrackingSample {
public:
    AltTrackingSample(JNIEnv* env, jobject instance);
    ~AltTrackingSample();

    std::string readOutput();

    float extrapolationTime = 0.06f;

private:
    Antilatency::DeviceNetwork::ILibrary _deviceNetworkLibrary;
    Antilatency::Alt::Tracking::ILibrary _altTrackingLibrary;
    Antilatency::Alt::Environment::Selector::ILibrary _altEnvironmentSelectorLibrary;
    Antilatency::StorageClient::ILibrary _storageClientLibrary;

    Antilatency::DeviceNetwork::INetwork _network;

    // Each time the device network is changed due to connection or disconnection of a device that matches the device filter of the network,
    // or start or stop of a task on any network device, the network update id is incremented by 1.
    uint32_t _prevUpdateId = 0;

    Antilatency::Alt::Environment::IEnvironment _altEnvironment;
    Antilatency::Math::floatP3Q _placement;
    Antilatency::Alt::Tracking::ITrackingCotask _altTrackingCotask;

    std::string _output;
    std::mutex _outputMutex;

    std::thread _altTrackingThread;
    bool _trackingThreadRunning = false;

    static void initJni(Antilatency::InterfaceContract::IInterface obj, JavaVM* jvm, jobject instance);

    void writeOutput(const std::string& text);
    void doTracking();
};



#endif //ANTILATENCY_ALTTRACKINGDEMOANDROID_ALTTRACKINGSAMPLE_H
