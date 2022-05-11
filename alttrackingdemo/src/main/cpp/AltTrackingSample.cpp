#include "AltTrackingSample.h"

#include <unistd.h>
#include <sstream>
#include <memory>

#include <Antilatency.InterfaceContract.LibraryLoader.h>

std::unique_ptr<AltTrackingSample> trackingInstance;

JNIEXPORT void JNICALL
Java_com_antilatency_alttrackingdemo_MainActivity_Init(JNIEnv* env, jobject instance){
    if (trackingInstance == nullptr){
        trackingInstance = std::make_unique<AltTrackingSample>(env, instance);
    }
}

JNIEXPORT jstring JNICALL
Java_com_antilatency_alttrackingdemo_MainActivity_GetOutput(JNIEnv* env, jobject instance){
    if (trackingInstance == nullptr){
        return (jstring)"Fail";
    }

    return env->NewStringUTF((trackingInstance->readOutput()).c_str());
}

AltTrackingSample::AltTrackingSample(JNIEnv *env, jobject instance) {
    // Load the Antilatency Device Network library
    _deviceNetworkLibrary = Antilatency::InterfaceContract::getLibraryInterface<Antilatency::DeviceNetwork::ILibrary>("libAntilatencyDeviceNetwork.so");
    if (_deviceNetworkLibrary == nullptr){
        throw std::runtime_error("Failed to load Device Network library");
    }

    // Load the Antilatency Alt Tracking library
    _altTrackingLibrary = Antilatency::InterfaceContract::getLibraryInterface<Antilatency::Alt::Tracking::ILibrary>("libAntilatencyAltTracking.so");
    if (_altTrackingLibrary == nullptr){
        throw std::runtime_error("Failed to load Alt Tracking library");
    }

    // Load the Antilatency Alt Environment Selector library
    _altEnvironmentSelectorLibrary = Antilatency::InterfaceContract::getLibraryInterface<Antilatency::Alt::Environment::Selector::ILibrary>("libAntilatencyAltEnvironmentSelector.so");
    if (_altEnvironmentSelectorLibrary == nullptr){
        throw std::runtime_error("Failed to load Alt Environment Selector library");
    }

    // Load the Antilatency Storage Client library
    _storageClientLibrary = Antilatency::InterfaceContract::getLibraryInterface<Antilatency::StorageClient::ILibrary>("libAntilatencyStorageClient.so");
    if (_storageClientLibrary == nullptr){
        throw std::runtime_error("Failed to load Alt Storage Client library");
    }

    JavaVM* jvm;
    env->GetJavaVM(&jvm);

    initJni(_deviceNetworkLibrary, jvm, instance);
    initJni(_altTrackingLibrary, jvm, instance);
    initJni(_altEnvironmentSelectorLibrary, jvm, instance);
    initJni(_storageClientLibrary, jvm, instance);

    writeOutput("Antilatency Device Network library ver. " + _deviceNetworkLibrary.getVersion());

    // Create a device network filter and then create a network using that filter.
    Antilatency::DeviceNetwork::IDeviceFilter deviceFilter = _deviceNetworkLibrary.createFilter();
    deviceFilter.addUsbDevice(Antilatency::DeviceNetwork::Constants::AllUsbDevices);
    deviceFilter.addIpDevice(Antilatency::DeviceNetwork::Constants::AllIpDevicesIp, Antilatency::DeviceNetwork::Constants::AllIpDevicesMask);
    _network = _deviceNetworkLibrary.createNetwork(deviceFilter);

    // Get environment serialized data from Antilatency Service. Using "default" as key will return environment that marked as default in Antilatency Service.
    std::string environmentCode = _storageClientLibrary.getLocalStorage().read("environment", "default");
    // Get environment name from Antilatency Service
    std::string environmentName = _storageClientLibrary.getLocalStorage().read("environment", ".default");
    // Create environment object from the serialized data.
    _altEnvironment = _altEnvironmentSelectorLibrary.createEnvironment(environmentCode);
    if (_altEnvironment == nullptr){
        throw std::runtime_error("Failed to create environment");
    }

    // Get markers positions from the environment
    std::vector<Antilatency::Math::float3> markersPositions = _altEnvironment.getMarkers();
    writeOutput("Environment" + environmentName + " has been created, markers count: " + std::to_string(markersPositions.size()));

    for (auto marker : markersPositions){
        writeOutput("Marker: { " + std::to_string(marker.x) + ", " + std::to_string(marker.y) + ", " + std::to_string(marker.z) + "}");
    }

    // Get placement serialized data from Antilatency Service. Using "default" as key will return placement that marked as default in Antilatency Service.
    std::string placementCode = _storageClientLibrary.getLocalStorage().read("placement", "default");
    // Create placement from the serialized data.
    _placement = _altTrackingLibrary.createPlacement(placementCode);

    writeOutput("Placement offset: (" + std::to_string(_placement.position.x) + ", " + std::to_string(_placement.position.y) + ", " + std::to_string(_placement.position.z) + ")");
    writeOutput("Placement rotation: (" + std::to_string(_placement.rotation.x) + ", " + std::to_string(_placement.rotation.y) + ", " + std::to_string(_placement.rotation.z) + ", " + std::to_string(_placement.rotation.w) + ")");

    _trackingThreadRunning = true;
    _altTrackingThread = std::thread(&AltTrackingSample::doTracking, this);
}

AltTrackingSample::~AltTrackingSample() {
    _trackingThreadRunning = false;
    _altTrackingThread.join();
}

std::string AltTrackingSample::readOutput() {
    std::lock_guard<std::mutex> lock(_outputMutex);
    auto output = _output;
    _output = "";
    return output;
}

void AltTrackingSample::initJni(Antilatency::InterfaceContract::IInterface obj, JavaVM* jvm, jobject instance) {
    auto jni = obj.queryInterface<AndroidJniWrapper::IAndroidJni>();
    if (jni == nullptr){
        return;
    }

    jni.initJni(jvm, instance);
}

void AltTrackingSample::writeOutput(const std::string &text) {
    std::lock_guard<std::mutex> lock(_outputMutex);
    _output += text + "\n";
}

void AltTrackingSample::doTracking() {
    while (_trackingThreadRunning){
        if (_altTrackingCotask != nullptr && _altTrackingCotask.isTaskFinished()){
            _altTrackingCotask = {};
        }

        if (_altTrackingCotask == nullptr){
            // Check if the network has been changed.
            uint32_t currentUpdateId = _network.getUpdateId();
            if (_prevUpdateId != currentUpdateId){
                _prevUpdateId = currentUpdateId;
                writeOutput("Antilatency Device Network update id has been incremented: " + std::to_string(currentUpdateId));
                writeOutput("Searching for idle nodes that supports tracking task...");

                // Create alt tracking cotask constructor to find tracking-supported nodes and start tracking task on node.
                Antilatency::Alt::Tracking::ITrackingCotaskConstructor cotaskConstructor = _altTrackingLibrary.createTrackingCotaskConstructor();
                std::vector<Antilatency::DeviceNetwork::NodeHandle> availableTrackingNodes = cotaskConstructor.findSupportedNodes(_network);

                // Get first idle node that supports tracking task.
                for (auto node : availableTrackingNodes){
                    std::stringstream ss;
                    ss << static_cast<std::underlying_type<Antilatency::DeviceNetwork::NodeHandle>::type>(node);
                    if (_network.nodeGetStatus(node) == Antilatency::DeviceNetwork::NodeStatus::Idle){
                        writeOutput("Tracking node found, node: " + ss.str());
                        _altTrackingCotask = cotaskConstructor.startTask(_network, node, _altEnvironment);
                        break;
                    }
                }

                if (_altTrackingCotask == nullptr){
                    writeOutput("Tracking node not found");
                }
            }
        }

        if (_altTrackingCotask != nullptr){
            Antilatency::Alt::Tracking::State extrapolatedState = _altTrackingCotask.getExtrapolatedState(_placement, extrapolationTime);

            writeOutput("State:");

            writeOutput("\tPose:");
            writeOutput(
                    "\t\tPosition: x: " + std::to_string(extrapolatedState.pose.position.x) +
                    ", y: " + std::to_string(extrapolatedState.pose.position.y) +
                    ", z: " + std::to_string(extrapolatedState.pose.position.z)
            );
            writeOutput(
                    "\t\tRotation: x: " + std::to_string(extrapolatedState.pose.rotation.x) +
                    ", y: " + std::to_string(extrapolatedState.pose.rotation.y) +
                    ", z: " + std::to_string(extrapolatedState.pose.rotation.z) +
                    ", w: " + std::to_string(extrapolatedState.pose.rotation.w)
            );

            writeOutput("\tStability:");
            writeOutput("\t\tStage: " + std::to_string(static_cast<int32_t>(extrapolatedState.stability.stage)));
            writeOutput("\t\tValue: " + std::to_string(extrapolatedState.stability.value));

            writeOutput(
                    "\tVelocity: x: " +
                    std::to_string(extrapolatedState.velocity.x) +
                    ", y: " +
                    std::to_string(extrapolatedState.velocity.y) +
                    ", z: " +
                    std::to_string(extrapolatedState.velocity.z)
            );

            writeOutput(
                    "\tLocalAngularVelocity: x: " +
                    std::to_string(extrapolatedState.localAngularVelocity.x) +
                    ", y: " +
                    std::to_string(extrapolatedState.localAngularVelocity.y) +
                    ", z: " +
                    std::to_string(extrapolatedState.localAngularVelocity.z)
            );
        }
    }
}
