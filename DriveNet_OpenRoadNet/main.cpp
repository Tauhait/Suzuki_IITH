/////////////////////////////////////////////////////////////////////////////////////////
// This code contains NVIDIA Confidential Information and is disclosed
// under the Mutual Non-Disclosure Agreement.
//
// Notice
// ALL NVIDIA DESIGN SPECIFICATIONS AND CODE ("MATERIALS") ARE PROVIDED "AS IS" NVIDIA MAKES
// NO REPRESENTATIONS, WARRANTIES, EXPRESSED, IMPLIED, STATUTORY, OR OTHERWISE WITH RESPECT TO
// THE MATERIALS, AND EXPRESSLY DISCLAIMS ANY IMPLIED WARRANTIES OF NONINFRINGEMENT,
// MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE.
//
// NVIDIA Corporation assumes no responsibility for the consequences of use of such
// information or for any infringement of patents or other rights of third parties that may
// result from its use. No license is granted by implication or otherwise under any patent
// or patent rights of NVIDIA Corporation. No third party distribution is allowed unless
// expressly authorized by NVIDIA.  Details are subject to change without notice.
// This code supersedes and replaces all information previously supplied.
// NVIDIA Corporation products are not authorized for use as critical
// components in life support devices or systems without express written approval of
// NVIDIA Corporation.
//
// Copyright (c) 2015-2019 NVIDIA Corporation. All rights reserved.
//
// NVIDIA Corporation and its licensors retain all intellectual property and proprietary
// rights in and to this software and related documentation and any modifications thereto.
// Any use, reproduction, disclosure or distribution of this software and related
// documentation without an express license agreement from NVIDIA Corporation is
// strictly prohibited.
//
/////////////////////////////////////////////////////////////////////////////////////////

#define _CRT_SECURE_NO_WARNINGS///

#include <thread>///

// Samples
#include <framework/DriveWorksSample.hpp>
#include <framework/SimpleStreamer.hpp>
#include <framework/SampleFramework.hpp>
#include <framework/SimpleCamera.hpp>
#include <framework/Checks.hpp>///
#include <framework/DataPath.hpp>///
#include <framework/Log.hpp>///
#include <framework/ProgramArguments.hpp>///

//ipc
#include <signal.h>
#include <iostream>
#include <thread>

// Core
#include <dw/core/Context.h>
#include <dw/core/Logger.h>
#include <dw/core/VersionCurrent.h>
#include <dw/comms/socketipc/SocketClientServer.h>///

// HAL
#include <dw/sensors/Sensors.h>

// DriveNet
#include <dw/dnn/DriveNet.h>
#include <dw/perception/object/camera/ObjectDetector.h>
#include <dw/world/ObjectArray.h>

// Renderer
#include <dwvisualization/core/RenderEngine.h>

#include <iomanip> // setprecision
#include <sstream> // stringstream

// IMAGE
#include <dw/interop/streamer/ImageStreamer.h>

// FreespaceDetector
#include <dw/perception/freespace/camera/FreespaceDetector.h>///
#include <dw/dnn/OpenRoadNet.h>///

// Calibrated camera
#include <dw/rig/Rig.h>///
#include <dw/calibration/cameramodel/CameraModel.h>///

//File operations
#include <iostream>
#include <fstream>

#include <string>
#include "SocketConnection.cpp"

using namespace dw_samples::common;

class DriveNetSimpleApp : public DriveWorksSample
{
private:
    // ------------------------------------------------
    // Driveworks Context, SAL and render engine
    // ------------------------------------------------
    dwContextHandle_t m_context           = DW_NULL_HANDLE;
    dwVisualizationContextHandle_t m_viz  = DW_NULL_HANDLE;
    dwRenderEngineHandle_t m_renderEngine = DW_NULL_HANDLE;
    dwSALHandle_t m_sal                   = DW_NULL_HANDLE;

    // ------------------------------------------------
    // DriveNet
    // ------------------------------------------------
    uint32_t m_numImages          = 1U;
    dwDriveNetHandle_t m_driveNet = DW_NULL_HANDLE;
    dwDriveNetParams m_driveNetParams{};
    // dwDriveNetClassProperties m_driveNetClassProperties{};
    const uint32_t m_maxProposalsPerClass    = 1000U;
    const uint32_t m_maxClustersPerClass     = 400U;
    // Detector
    dwObjectDetectorParams m_detectorParams{};
    dwObjectDetectorHandle_t m_driveNetDetector = DW_NULL_HANDLE;
    // Colors for rendering bounding boxes
    static const uint32_t MAX_BOX_COLORS         = DW_OBJECT_CLASS_NUM_CLASSES;
    const dwVector4f m_boxColors[MAX_BOX_COLORS] = {{1.0f, 0.0f, 0.0f, 1.0f},
                                                    {0.0f, 1.0f, 0.0f, 1.0f},
                                                    {0.0f, 0.0f, 1.0f, 1.0f},
                                                    {1.0f, 0.0f, 1.0f, 1.0f},
                                                    {1.0f, 0.647f, 0.0f, 1.0f}};
    // Labels of each class
    std::vector<std::string> m_classLabels;
    // Vectors of boxes and class label ids
    std::vector<std::vector<dwRectf>> m_dnnBoxList;
    std::vector<std::vector<std::string>> m_dnnLabelList;
    std::vector<std::vector<const char*>> m_dnnLabelListPtr;
    // Vector of urgency value and whether it is valid
    std::vector<std::vector<float>> m_urgencyList;
    std::vector<std::vector<bool>> m_isUrgencyValidList;
    // Detector ROIs
    dwRectf m_detectorROIs[2];

    // ------------------------------------------------
    // Renderer
    // ------------------------------------------------
    dwImageHandle_t m_imageRGBA;
    std::unique_ptr<SimpleImageStreamerGL<>> m_streamerCUDA2GL;
    cudaStream_t m_cudaStream = 0;

    // ------------------------------------------------
    // Camera
    // ------------------------------------------------
    std::unique_ptr<SimpleCamera> m_camera;
    dwImageGL* m_imgGl;
    dwImageProperties m_rcbProperties;
#ifdef VIBRANTE
    dwSensorHandle_t m_cameraMaster = DW_NULL_HANDLE;
#endif
    // image width and height
    uint32_t m_imageWidth;
    uint32_t m_imageHeight;
    bool m_isRaw;

    /// The maximum number of output objects for a given bound output.
    static constexpr uint32_t MAX_OBJECT_OUTPUT_COUNT = 1000 * DW_OBJECT_CLASS_NUM_CLASSES;

    dwObjectArray m_detectorOutput;

public:
    //------------------------------------------------------------------------------
    // Variables
    //------------------------------------------------------------------------------
    
    
    //for writing into file    
    std::ofstream objectFileWriter;
    std::ofstream boundaryImagePointFileWriter;
    std::ofstream boundaryWorldPointFileWriter;
    std::ofstream speedFileWriter;
    std::ofstream urgencyFileWriter;

    dwFreespaceDetectorHandle_t m_freespaceDetector = DW_NULL_HANDLE;///
    dwOpenRoadNetHandle_t m_openRoadNet           = DW_NULL_HANDLE;///
    dwFreespaceDetectorInitParams m_freespaceDetectorParams{};///
    dwFreespaceDetection m_freespaceBoundary{};///
    dwImageProperties m_cameraImageProperties;///
    dwRigHandle_t m_rigConfig          = DW_NULL_HANDLE;///
    dwCameraModelHandle_t m_calibratedCam      = DW_NULL_HANDLE;///
    uint32_t m_cameraWidth  = 0U;///
    uint32_t m_cameraHeight = 0U;///
    float32_t m_temporalSmoothFactor = 0.5f;///
    uint32_t m_spatialSmoothFilterWidth = 5;///
    float32_t m_drawScaleX;///
    float32_t m_drawScaleY;///
    std::string gInputType;///
    // bool m_isRaw = false;///
    bool m_rig = false;///

    // dwSocketConnectionHandle_t socketConnectionWrite = dwSocketConnectionHandle_t{DW_NULL_HANDLE};///
    // dwSocketConnectionHandle_t socketConnectionRead  = dwSocketConnectionHandle_t{DW_NULL_HANDLE};///
    // dwSocketClientHandle_t socketClient              = dwSocketClientHandle_t{DW_NULL_HANDLE};///
    // dwSocketClientHandle_t socketReceive             = dwSocketClientHandle_t{DW_NULL_HANDLE};///
    dwStatus status                                  = DW_FAILURE;///
    // dwContextHandle_t ctx                            = DW_NULL_HANDLE;
    
    SocketConnection pegasusReceiveSocket, pegasusSentSocket;
    const std::string receiveIP =  "192.168.50.2";
    const std::string sendIP    =  "192.168.50.1";
    const int receivePort = 51001;
    const int sendPort = 30000;
    

    /// -----------------------------
    /// Initialize application
    /// -----------------------------
    DriveNetSimpleApp(const ProgramArguments& args): DriveWorksSample(args) {}

    /// -----------------------------
    /// Initialize SDK, SAL, Sensors, Image Streamers, Renderer, DriveNet
    /// -----------------------------

    std::pair<int, int> setAngle(double m_nAngle, double deltaAngle){
        m_nAngle = m_nAngle + deltaAngle;
        double gearRatio = 17.75;
        if (40 * gearRatio < m_nAngle){
            m_nAngle = 40 * gearRatio;
        }else if (-40 * gearRatio > m_nAngle){
            m_nAngle = -40 * gearRatio;
        }
        double l_usAngle = (m_nAngle / gearRatio - (-65.536)) / 0.002;
        int H_Angle = (int)(l_usAngle) >> 8;
        int L_Angle = (int)(l_usAngle) & 0xff;
        return std::make_pair(H_Angle,L_Angle);
    }
    int getCheckSum(int size, int* msg){
        int checksum = 0;
        for(int i = 0; i < size; i++){
            checksum += msg[i];
        }
        checksum = (0x00 - checksum) & 0x000000FF;
        checksum = checksum & 0xFF;
        return checksum;
    }
    std::pair<int, int> setSpeed(double speed){
        speed = speed * 128;
        int H_speed = (int)(speed) >> 8;
        int L_speed = (int)(speed) & 0xff;
        return std::make_pair(H_speed, L_speed);
    }
    std::vector<uint8_t> buildMsgMABX(double speed, double m_nAngle, double angle, int flasher, int counter){
        std::vector<uint8_t> byteMsgArray;
        std::pair<int,int> steerAngle = setAngle(m_nAngle, -1 * angle);
        std::pair<int,int> currSpeed  = setSpeed(speed);
        int msg_list[] = {  1, counter, 0, 1, 52, 136, 215, 1, currSpeed.first, 
                            currSpeed.second, steerAngle.first, steerAngle.second, 0, flasher, 0, 0, 0, 0};
        msg_list[2] = getCheckSum(sizeof(msg_list), msg_list);
        for(int i = 0; i < 18; i++){
            char temp[8];
            std::sprintf(temp, "%02X", msg_list[i]);
            uint8_t msgbyte = (uint8_t)atoi(temp);
            byteMsgArray.push_back(msgbyte);
        }
        return byteMsgArray;
    }
    bool onInitialize() override
    {

        if ( ! pegasusReceiveSocket.create() )
        {
            return false;
        }
        if ( ! pegasusSentSocket.create() )
        {
            return false;
        }

        if ( ! pegasusReceiveSocket.connect ( receiveIP, receivePort ) )
        {
            return false;
        }
        if ( ! pegasusSentSocket.connect ( sendIP, sendPort ) )
        {
            return false;
        }
        std::cout<<"sent and receive client sockets created"<<std::endl;

        //-------Open File-------------------------
        { 
            if(objectFileWriter){
                objectFileWriter.open("object.csv");            
            }
            if(boundaryImagePointFileWriter){
                boundaryImagePointFileWriter.open("boundaryImagePoint.csv");            
            }
            if(boundaryWorldPointFileWriter){
                boundaryWorldPointFileWriter.open("boundaryWorldPoint.csv");            
            }
            if(speedFileWriter){
                speedFileWriter.open("speed.csv");            
            }
            if(urgencyFileWriter){
                urgencyFileWriter.open("urgency.csv");
            }
        }
        // -----------------------------------------
        // Initialize Cuda stream
        // -----------------------------------------
        {
            CHECK_CUDA_ERROR(cudaStreamCreateWithFlags(&m_cudaStream, cudaStreamNonBlocking));
        }

        // -----------------------------------------
        // Initialize DriveWorks SDK context and SAL
        // -----------------------------------------
        {
            initializeDriveWorks(m_context);
            CHECK_DW_ERROR(dwSAL_initialize(&m_sal, m_context));
        }
        std::cout << "after initializeDriveWorks()" << std::endl;
        //------------------------------------------------------------------------------
        // initialize Renderer
        //------------------------------------------------------------------------------
        {
            CHECK_DW_ERROR(dwVisualizationInitialize(&m_viz, m_context));

            // Setup render engine
            dwRenderEngineParams params{};
            CHECK_DW_ERROR(dwRenderEngine_initDefaultParams(&params, getWindowWidth(), getWindowHeight()));
            params.defaultTile.lineWidth = 0.2f;
            params.defaultTile.font      = DW_RENDER_ENGINE_FONT_VERDANA_20;
            CHECK_DW_ERROR(dwRenderEngine_initialize(&m_renderEngine, &params, m_viz));
        }

        std::cout << "after dwRenderEngine_initialize()" << std::endl;
        //------------------------------------------------------------------------------
        // initialize Sensors
        //------------------------------------------------------------------------------
        {
            dwSensorParams params{};
            {
#ifdef VIBRANTE
                if (getArgument("input-type").compare("camera") == 0)
                {
                    std::string parameterString = "camera-type=" + getArgument("camera-type");
                    parameterString += ",camera-group=" + getArgument("camera-group");
                    parameterString += ",slave=" + getArgument("slave");
                    parameterString += ",serialize=false,output-format=raw,camera-count=4";
                    std::string cameraMask[4] = {"0001", "0010", "0100", "1000"};
                    uint32_t cameraIdx        = std::stoi(getArgument("camera-index"));
                    if (cameraIdx < 0 || cameraIdx > 3)
                    {
                        std::cerr << "Error: camera index must be 0, 1, 2 or 3" << std::endl;
                        return false;
                    }
                    parameterString += ",camera-mask=" + cameraMask[cameraIdx];

                    params.parameters = parameterString.c_str();
                    params.protocol   = "camera.gmsl";

                    //m_camera.reset(new RawSimpleCamera(params, m_sal, m_context, m_cudaStream, DW_CAMERA_OUTPUT_NATIVE_PROCESSED));
                    m_camera.reset(new RawSimpleCamera(DW_IMAGE_FORMAT_RGBA_UINT8, params, m_sal, m_context,
                                                       m_cudaStream, DW_CAMERA_OUTPUT_NATIVE_PROCESSED,
                                                       DW_SOFTISP_DEMOSAIC_METHOD_INTERPOLATION));///
                    m_isRaw = true;
                }
                else if (getArgument("input-type").compare("cameraCustom") == 0)
                {
                    dwSensorParams paramsMaster{};
                    std::string parameterStringMaster = getArgument("cameraCustomString");

                    paramsMaster.parameters = parameterStringMaster.c_str();
                    paramsMaster.protocol   = "camera.gmsl.master";

                    CHECK_DW_ERROR(dwSAL_createSensor(&m_cameraMaster, paramsMaster, m_sal));


                    std::string parameterStringClient = "output-format=processed,camera-id=0";
                    params.parameters = parameterStringClient.c_str();
                    params.protocol   = "camera.gmsl.client";

                    m_camera.reset(new SimpleCamera(params, m_sal, m_context));
                    dwImageProperties outputProperties = m_camera->getOutputProperties();
                    outputProperties.type              = DW_IMAGE_CUDA;
                    outputProperties.format            = DW_IMAGE_FORMAT_RGB_FLOAT16_PLANAR;
                    m_camera->setOutputProperties(outputProperties);

                    std::cout << "Successfully initialized with custom camera" << std::endl;

                }
                else
#endif
                {
                    std::string parameterString = getArgs().parameterString();
                    params.parameters           = parameterString.c_str();
                    params.protocol             = "camera.virtual";

                    std::string videoFormat = getArgument("video");
                    std::size_t found       = videoFormat.find_last_of(".");

                    if (videoFormat.substr(found + 1).compare("h264") == 0 || videoFormat.substr(found + 1).compare("mp4") == 0)
                    {
                        m_camera.reset(new SimpleCamera(params, m_sal, m_context));
                        dwImageProperties outputProperties = m_camera->getOutputProperties();
                        outputProperties.type              = DW_IMAGE_CUDA;
                        outputProperties.format            = DW_IMAGE_FORMAT_RGB_FLOAT16_PLANAR;
                        // outputProperties.format            = DW_IMAGE_FORMAT_RGBA_UINT8;///
                        m_camera->setOutputProperties(outputProperties);
                        m_isRaw = false;
                    }
                    else
                    {
                        // m_camera.reset(new RawSimpleCamera(params, m_sal, m_context, m_cudaStream,
                        //                                    DW_CAMERA_OUTPUT_NATIVE_PROCESSED));
                        m_camera.reset(new RawSimpleCamera(DW_IMAGE_FORMAT_RGBA_UINT8, params, m_sal, m_context, m_cudaStream,
                                                           DW_CAMERA_OUTPUT_NATIVE_PROCESSED, DW_SOFTISP_DEMOSAIC_METHOD_INTERPOLATION));///
                    }
                }
            }

            if (m_camera == nullptr)
            {
                logError("Camera could not be created\n");
                return false;
            }

#ifdef VIBRANTE
            if (getArgument("input-type").compare("camera") == 0)
            {
                dwCameraRawFormat rawFormat = m_camera->getCameraProperties().rawFormat;
                if (rawFormat != DW_CAMERA_RAW_FORMAT_RCCB &&
                    rawFormat != DW_CAMERA_RAW_FORMAT_BCCR &&
                    rawFormat != DW_CAMERA_RAW_FORMAT_CRBC &&
                    rawFormat != DW_CAMERA_RAW_FORMAT_CBRC)
                {
                    logError("Camera not supported, see documentation\n");
                    return false;
                }
            }
#endif

            std::cout << "Camera image with " << m_camera->getCameraProperties().resolution.x << "x"
                      << m_camera->getCameraProperties().resolution.y << " at "
                      << m_camera->getCameraProperties().framerate << " FPS" << std::endl;

            dwImageProperties displayProperties = m_camera->getOutputProperties();
            displayProperties.format            = DW_IMAGE_FORMAT_RGBA_UINT8;

            CHECK_DW_ERROR(dwImage_create(&m_imageRGBA, displayProperties, m_context));

            m_streamerCUDA2GL.reset(new SimpleImageStreamerGL<>(displayProperties, 1000, m_context));

            m_rcbProperties = m_camera->getOutputProperties();

            m_imageWidth  = displayProperties.width;
            m_imageHeight = displayProperties.height;
            m_cameraWidth = m_camera->getCameraProperties().resolution.x;///
            m_cameraHeight = m_camera->getCameraProperties().resolution.y;///
        }
        
        std::cout << "after initialize sensors" << std::endl;
        //initialize rig config-------------------------------------------
        {
            m_rig = initRigConfiguration();///
        }

        //------------------------------------------------------------------------------
        // initialize DriveNet detector
        //------------------------------------------------------------------------------
        {
            // Initialize DriveNet network
            CHECK_DW_ERROR(dwDriveNet_initDefaultParams(&m_driveNetParams));
            // Set up max number of proposals and clusters
            m_driveNetParams.maxClustersPerClass  = m_maxClustersPerClass;
            m_driveNetParams.maxProposalsPerClass = m_maxProposalsPerClass;
            m_driveNetParams.batchSize            = DW_DRIVENET_BATCH_SIZE_1;
            bool enableUrgency = getArgument("enableUrgency").compare("1") == 0;
            bool useStatelessModel = getArgument("stateless").compare("1") == 0;
            if (enableUrgency)
            {
                m_driveNetParams.hasObjectUrgency = true;

                // Use the temporal DriveNet model that also predicts urgency, in addition to
                // the regular DriveNet predictions.
                m_driveNetParams.networkModel = useStatelessModel ? DW_STATELESS_TEMPORAL_DRIVENET_MODEL : DW_STATEFUL_TEMPORAL_DRIVENET_MODEL;
            }
            else
            {
                m_driveNetParams.networkModel = DW_DRIVENET_MODEL_FRONT;
            }
            
            // Get precision from command line
            std::string precisionArg = getArgument("precision");
            if (precisionArg.compare("fp32") == 0)
            {
                m_driveNetParams.networkPrecision = DW_PRECISION_FP32;
            }
            else if (precisionArg.compare("fp16") == 0)
            {
                m_driveNetParams.networkPrecision = DW_PRECISION_FP16;
            }
            else if (precisionArg.compare("int8") == 0)
            {
                m_driveNetParams.networkPrecision = DW_PRECISION_INT8;
            }
            else
            {
                std::cerr << "Unknown DriveNet model precision." << std::endl;
                return false;
            }

            // Check if network should run on DLA
            bool dla = getArgument("dla").compare("1") == 0;

            // Check if this platform supports DLA
            if (dla)
            {
                int32_t dlaEngineCount = 0;
                CHECK_DW_ERROR(dwContext_getDLAEngineCount(&dlaEngineCount, m_context));
                if (!dlaEngineCount)
                {
                    throw std::runtime_error("No DLA Engine available on this platform.");
                }

                // Check which DLA engine should DriveNet run on
                int32_t dlaEngineNo = std::atoi(getArgument("dlaEngineNo").c_str());

                switch (dlaEngineNo)
                {
                case 0:
                    m_driveNetParams.processorType = DW_PROCESSOR_TYPE_DLA_0;
                    break;
                case 1:
                    m_driveNetParams.processorType = DW_PROCESSOR_TYPE_DLA_1;
                    break;
                default:
                    throw std::runtime_error("DLA Engine no must be either 0 or 1.");
                }
                // DLA supports only FP16 precision.
                m_driveNetParams.networkPrecision = DW_PRECISION_FP16;
            }

            CHECK_DW_ERROR(dwDriveNet_initialize(&m_driveNet, &m_driveNetParams, m_context));

            // Initialize Object Detector from DriveNet
            CHECK_DW_ERROR(dwObjectDetector_initDefaultParams(&m_detectorParams));
            m_detectorParams.maxNumImages = m_numImages;

            CHECK_DW_ERROR(dwObjectDetector_initializeFromDriveNet(&m_driveNetDetector, &m_detectorParams,
                                                                   m_driveNet, m_context));

            CHECK_DW_ERROR(dwObjectDetector_setCUDAStream(m_cudaStream, m_driveNetDetector));

            // since our input images might have a different aspect ratio as the input to drivenet
            // we setup the ROI such that the crop happens from the top of the image
            float32_t aspectRatio = 1.0f;
            {
                dwBlobSize inputBlob;
                CHECK_DW_ERROR(dwDriveNet_getInputBlobsize(&inputBlob, m_driveNet));

                aspectRatio = static_cast<float32_t>(inputBlob.height) / static_cast<float32_t>(inputBlob.width);
            }

            // 1st image is a full resolution image as it comes out from the RawPipeline (cropped to DriveNet aspect ratio)
            dwRect fullROI;
            {
                fullROI = {0, 0, static_cast<int32_t>(m_rcbProperties.width),
                           static_cast<int32_t>(m_rcbProperties.width * aspectRatio)};
                dwTransformation2f transformation = {{1.0f, 0.0f, 0.0f,
                                                      0.0f, 1.0f, 0.0f,
                                                      0.0f, 0.0f, 1.0f}};

                CHECK_DW_ERROR(dwObjectDetector_setROI(0, &fullROI, &transformation, m_driveNetDetector));
            }

            // fill out member structure according to the ROIs
            for (uint32_t roiIdx = 0U; roiIdx < m_numImages; ++roiIdx)
            {
                CHECK_DW_ERROR(dwObjectDetector_getROI(&m_detectorParams.ROIs[roiIdx],
                                                       &m_detectorParams.transformations[roiIdx], roiIdx, m_driveNetDetector));
                m_detectorROIs[roiIdx].x      = m_detectorParams.ROIs[roiIdx].x;
                m_detectorROIs[roiIdx].y      = m_detectorParams.ROIs[roiIdx].y;
                m_detectorROIs[roiIdx].width  = m_detectorParams.ROIs[roiIdx].width;
                m_detectorROIs[roiIdx].height = m_detectorParams.ROIs[roiIdx].height;
            }

            // Create object array to store the output
            dwObjectArray_create(&m_detectorOutput, MAX_OBJECT_OUTPUT_COUNT, DW_OBJECT_TYPE_CAMERA);

            // Initialize box list
            m_dnnBoxList.resize(DW_OBJECT_CLASS_NUM_CLASSES);
            m_dnnLabelList.resize(DW_OBJECT_CLASS_NUM_CLASSES);
            m_dnnLabelListPtr.resize(DW_OBJECT_CLASS_NUM_CLASSES);
            m_isUrgencyValidList.resize(DW_OBJECT_CLASS_NUM_CLASSES);
            m_urgencyList.resize(DW_OBJECT_CLASS_NUM_CLASSES);

            // Get which label name for each class id
            m_classLabels.resize(DW_OBJECT_CLASS_NUM_CLASSES);

            for (uint32_t classIdx = 0U; classIdx < DW_OBJECT_CLASS_NUM_CLASSES; ++classIdx)
            {
                // Clear the class label
                m_classLabels[classIdx] = "";

                // Reserve label and box lists
                m_dnnBoxList[classIdx].reserve(m_maxClustersPerClass);
                m_dnnLabelList[classIdx].reserve(m_maxClustersPerClass);
                m_dnnLabelListPtr[classIdx].reserve(m_maxClustersPerClass);
                m_isUrgencyValidList[classIdx].reserve(m_maxClustersPerClass);
                m_urgencyList[classIdx].reserve(m_maxClustersPerClass);
            }

            // Define labels for the known classes
            m_classLabels[DW_OBJECT_CLASS_CAR]           = "car";
            m_classLabels[DW_OBJECT_CLASS_BICYCLE]       = "bicycle";
            m_classLabels[DW_OBJECT_CLASS_PEDESTRIAN]    = "pedestrian";
            m_classLabels[DW_OBJECT_CLASS_TRAFFIC_LIGHT] = "traffic_light";
            m_classLabels[DW_OBJECT_CLASS_TRAFFIC_SIGN]  = "traffic_sign";
        }

        std::cout << "after drivenet detector interface" << std::endl;
#ifdef VIBRANTE
        if (getArgument("input-type").compare("cameraCustom") == 0)
        {
            CHECK_DW_ERROR(dwSensor_start(m_cameraMaster));
        }
#endif
        // return true;
        return initDNN();///
        std::cout << "after initDNN()" << std::endl;
    }

    //#######################################################################################
    bool initRigConfiguration()
    {
        dwStatus result = DW_SUCCESS;
        //Load vehicle configuration
        result = dwRig_initializeFromFile(&m_rigConfig, m_context, getArgument("rig").c_str());
        if (result != DW_SUCCESS) {
            logError("Error dwRig_initialize: ", dwGetStatusName(result));
            return false;
        }
        result = dwCameraModel_initialize(&m_calibratedCam, 0, m_rigConfig);
        if (result != DW_SUCCESS) {
            logError("Error dwCameraRig_initializeFromConfig: ", dwGetStatusName(result));
            return false;
        }
        return true;
    }

    //#######################################################################################
    bool initDNN()
    {
        dwOpenRoadNetParams openRoadNetParams{};
        CHECK_DW_ERROR(dwOpenRoadNet_initDefaultParams(&openRoadNetParams));
        openRoadNetParams.networkModel = DW_OPENROADNET_MODEL_FRONT;
        CHECK_DW_ERROR(dwOpenRoadNet_initialize(&m_openRoadNet, m_context, &openRoadNetParams));

        dwTransformation3f transformation{};
        CHECK_DW_ERROR(dwRig_getSensorToRigTransformation(&transformation, 0, m_rigConfig));

        // Get maximum distance in meters at which free space boundary distance can be distinguished from user input.
        float32_t maxDistance = 50.0f;
        std::string maxDistanceStr = getArgument("maxDistance");
        if(maxDistanceStr!="50.0") {
            try{
                maxDistance = std::stof(maxDistanceStr);
                if (maxDistance < 0.0f) {
                    logError("maxDistance cannot be negative.\n");
                    return false;
                }
            } catch(...) {
                logError("Given maxDistance can't be parsed\n");
                return false;
            }
        }
        m_freespaceDetectorParams.openroadnet    = m_openRoadNet;
        m_freespaceDetectorParams.frameWidth     = m_cameraWidth;
        m_freespaceDetectorParams.frameHeight    = m_cameraHeight;
        m_freespaceDetectorParams.stream         = m_cudaStream;
        m_freespaceDetectorParams.cam_new[0]     = m_calibratedCam;
        m_freespaceDetectorParams.maxDistance[0] = maxDistance;
        CHECK_DW_ERROR(dwFreespaceDetector_initializeFromOpenRoadNet_new(&m_freespaceDetector,
                                                                         &m_freespaceDetectorParams,
                                                                         m_context));

        CHECK_DW_ERROR(dwFreespaceDetector_setTemporalSmoothFactor(m_temporalSmoothFactor, m_freespaceDetector));
        CHECK_DW_ERROR(dwFreespaceDetector_setSpatialSmoothFilterWidth(m_spatialSmoothFilterWidth, m_freespaceDetector));
        CHECK_DW_ERROR(dwFreespaceDetector_setCameraExtrinsics_new(0U, transformation, m_freespaceDetector));

        // boundary points are in camera space
        m_drawScaleX = static_cast<float32_t>(getWindowWidth())/static_cast<float32_t>(m_cameraWidth);
        m_drawScaleY = static_cast<float32_t>(getWindowHeight())/static_cast<float32_t>(m_cameraHeight);

        return true;
    }

    ///------------------------------------------------------------------------------
    /// Main processing of the sample
    ///     - collect sensor frame
    ///     - run detection
    ///------------------------------------------------------------------------------
    void onProcess() override
    {
        // read from camera
        dwImageCUDA* rcbImage = nullptr;
        getNextFrame(&rcbImage, &m_imgGl);
        std::this_thread::yield();
        while (rcbImage == nullptr)
        {
            onReset();

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            getNextFrame(&rcbImage, &m_imgGl);
        }

        // detect objects and get the results
        CHECK_DW_ERROR(dwObjectDetector_detectObjects(&m_detectorOutput, rcbImage, m_driveNetDetector));
        runDetector(rcbImage);///
        processResults();
    }

    ///------------------------------------------------------------------------------
    /// Render sample output on screen
    ///     - render video
    ///     - render boxes with labels
    ///------------------------------------------------------------------------------
    void onRender() override
    {
        bool enableUrgency = getArgument("enableUrgency").compare("1") == 0;
        //###################################################################################################///
        float32_t maxWidth = 8.0; //10 meters as a step, [0, 10) will have max line width
        float32_t witdhRatio = 0.8;
        float32_t dist2Width[20];
        dist2Width[0] = maxWidth;
        for(uint32_t i = 1; i < 20; i++)
            dist2Width[i] = dist2Width[i-1]*witdhRatio;

        float32_t prevWidth, curWidth = maxWidth/2;
        if(m_rig) {
            prevWidth = dist2Width[static_cast<uint32_t>(m_freespaceBoundary.boundaryWorldPoint[0].x/10)];
        } else {
            prevWidth = curWidth;
        }
        // std::cout<<"prevWidth: "<<prevWidth<<std::endl;
        //###################################################################################################///
        CHECK_DW_ERROR(dwRenderEngine_setTile(0, m_renderEngine));
        CHECK_DW_ERROR(dwRenderEngine_resetTile(m_renderEngine));

        dwVector2f range{};
        range.x = m_imgGl->prop.width;
        range.y = m_imgGl->prop.height;
        CHECK_DW_ERROR(dwRenderEngine_setCoordinateRange2D(range, m_renderEngine));
        CHECK_DW_ERROR(dwRenderEngine_renderImage2D(m_imgGl, {0.0f, 0.0f, range.x, range.y}, m_renderEngine));

        for (size_t classIdx = 0; classIdx < m_classLabels.size(); classIdx++)
        // for (size_t classIdx = 0; classIdx <= 3; classIdx++)        
        {
            if(!(classIdx == 0 || classIdx == 2 || classIdx == 3)) continue;

            if (&m_dnnBoxList[classIdx][0] == nullptr)
                continue;

            dwVector4f color = m_boxColors[classIdx % MAX_BOX_COLORS];
            for (size_t boxIdx = 0; boxIdx < m_dnnBoxList[classIdx].size(); boxIdx++)
            {
                float urgency = 0;
                if (enableUrgency)
                {
                    color = {1.f, 1.f, 1.f, 1.f};
                    if (m_isUrgencyValidList[classIdx][boxIdx])
                    {
                        urgency = m_urgencyList[classIdx][boxIdx];
                        // use color to describe urgency
                        // color:   green    -> white -> red
                        // urgency: negative -> 0     -> positive
                        if(urgency > 0) // red
                        {
                            color.y = std::max(0.0f, 1.f - urgency);
                            color.z = std::max(0.0f, 1.f - urgency);


                        }
                        else            //green
                        {
                            color.x = std::max(0.0f, 1.f + urgency);
                            color.z = std::max(0.0f, 1.f + urgency);
                        }
                    }
                }
                // if(urgency > 1.5){
                //     const char * 	dangerText = "DANGER! Too Close!!";
                //     dwVector2f      dangerTextPos{250, 250};
                //     CHECK_DW_ERROR(dwRenderEngine_renderText2D(dangerText, dangerTextPos, m_renderEngine));
                // }
                CHECK_DW_ERROR(dwRenderEngine_setColor(color, m_renderEngine));
                CHECK_DW_ERROR(dwRenderEngine_renderWithLabels(DW_RENDER_ENGINE_PRIMITIVE_TYPE_BOXES_2D,
                                                                &m_dnnBoxList[classIdx][boxIdx],
                                                                sizeof(dwRectf),
                                                                0,
                                                                &m_dnnLabelListPtr[classIdx][boxIdx],
                                                                1,
                                                                m_renderEngine));
                std::vector<std::vector<dwRectf>> m_dnnBoxList;///
                std::vector<std::vector<std::string>> m_dnnLabelList;///
                for(std::vector<dwRectf> dnnBox : m_dnnBoxList){
                    for(dwRectf rect : dnnBox){                        
                        objectFileWriter << rect.x << "," << rect.y << "," << rect.height << "," << rect.width << "\n";
                    }
                }
            }
        }
        m_detectorROIs[0].height += 10;
        // draw ROI of the first image
        CHECK_DW_ERROR(dwRenderEngine_setColor(DW_RENDERER_COLOR_LIGHTBLUE, m_renderEngine));
        CHECK_DW_ERROR(dwRenderEngine_render(DW_RENDER_ENGINE_PRIMITIVE_TYPE_BOXES_2D, &m_detectorROIs[0], sizeof(dwRectf), 0, 1, m_renderEngine));

        // draw ROI of the second image
        CHECK_DW_ERROR(dwRenderEngine_setColor(DW_RENDERER_COLOR_YELLOW, m_renderEngine));
        CHECK_DW_ERROR(dwRenderEngine_render(DW_RENDER_ENGINE_PRIMITIVE_TYPE_BOXES_2D, &m_detectorROIs[1], sizeof(dwRectf), 0, 1, m_renderEngine));

        // renderutils::renderFPS(m_renderEngine, getCurrentFPS());///

        // FreeSpace Detection
        //#################################################################################################################################################
        uint32_t index = 0;
        uint32_t count = 1;
        CHECK_DW_ERROR(dwRenderEngine_setLineWidth(prevWidth, m_renderEngine));

        // render freespace boundary, chaging color per category
        for (uint32_t i = 1; i < m_freespaceBoundary.numberOfBoundaryPoints; ++i)
        {
            if(m_rig)
                curWidth = dist2Width[static_cast<uint32_t>(m_freespaceBoundary.boundaryWorldPoint[i].x/10)];
            // std::cout<<"Width: "<<curWidth<<std::endl;
            CHECK_DW_ERROR(dwRenderEngine_setLineWidth(curWidth, m_renderEngine));
            // std::cout<<"Boundry Type: " << m_freespaceBoundary.boundaryType[i-1]<<std::endl;///

            if (m_freespaceBoundary.boundaryType[i] != m_freespaceBoundary.boundaryType[i-1] ||
                    (curWidth != prevWidth && count > 1) || i == m_freespaceBoundary.numberOfBoundaryPoints-1)
            {
                dwFreespaceBoundaryType category = m_freespaceBoundary.boundaryType[i-1];
                if (category==DW_BOUNDARY_TYPE_OTHER) {
                    CHECK_DW_ERROR(dwRenderEngine_setColor(DW_RENDERER_COLOR_YELLOW, m_renderEngine));
                } else if (category==DW_BOUNDARY_TYPE_CURB) {
                    CHECK_DW_ERROR(dwRenderEngine_setColor(DW_RENDERER_COLOR_GREEN, m_renderEngine));
                } else if (category==DW_BOUNDARY_TYPE_VEHICLE) {
                    CHECK_DW_ERROR(dwRenderEngine_setColor(DW_RENDERER_COLOR_RED, m_renderEngine));
                } else if (category==DW_BOUNDARY_TYPE_PERSON) {
                    CHECK_DW_ERROR(dwRenderEngine_setColor(DW_RENDERER_COLOR_LIGHTBLUE, m_renderEngine));
                }
                CHECK_DW_ERROR(dwRenderEngine_render(DW_RENDER_ENGINE_PRIMITIVE_TYPE_LINESTRIP_2D,
                                                     &m_freespaceBoundary.boundaryImagePoint[index],
                                                     sizeof(dwVector2f),
                                                     0,
                                                     count,
                                                     m_renderEngine));

// ====================================================================================================================================================
                
                // std::cout<<"FreeSpace image Point: " <<m_freespaceBoundary.boundaryImagePoint[index]<<std::endl;///
                // std::cout<<"Boundry Category: " <<category<<std::endl;///
                index = i;
                count = 1;
                prevWidth = curWidth;

                // std::string boundaryImageStr = std::to_string(m_freespaceBoundary.boundaryImagePoint[index].x) + "," + 
                //                                std::to_string(m_freespaceBoundary.boundaryImagePoint[index].y);
                // const char *boundaryImageCharPtr = boundaryImageStr.c_str();

                // std::string boundaryWorldStr = std::to_string(m_freespaceBoundary.boundaryWorldPoint[i].x) + "," + 
                //                                std::to_string(m_freespaceBoundary.boundaryWorldPoint[i].y);
                // const char *boundaryWorldCharPtr = boundaryWorldStr.c_str();

                // CHECK_DW_ERROR(dwRenderEngine_setColor(DW_RENDERER_COLOR_LIGHTPURPLE, m_renderEngine));
                // CHECK_DW_ERROR(dwRenderEngine_renderWithLabels(DW_RENDER_ENGINE_PRIMITIVE_TYPE_LINESTRIP_2D,
                //                                                 &m_freespaceBoundary.boundaryImagePoint[index],
                //                                                 sizeof(dwVector2f),
                //                                                 0,
                //                                                 &boundaryImageCharPtr,
                //                                                 count,
                //                                                 m_renderEngine));
                // CHECK_DW_ERROR(dwRenderEngine_setColor(DW_RENDERER_COLOR_ORANGE, m_renderEngine));
                // CHECK_DW_ERROR(dwRenderEngine_renderWithLabels(DW_RENDER_ENGINE_PRIMITIVE_TYPE_LINESTRIP_2D,
                //                                                 &m_freespaceBoundary.boundaryWorldPoint[index],
                //                                                 sizeof(dwVector2f),
                //                                                 0,
                //                                                 &boundaryWorldCharPtr,
                //                                                 1,
                //                                                 m_renderEngine));
                
// ====================================================================================================================================================
                // CHECK_DW_ERROR(dwRenderEngine_renderWithLabels(DW_RENDER_ENGINE_PRIMITIVE_TYPE_LINES_2D,
                //                                                 &m_freespaceBoundary.boundaryWorldPoint[i],
                //                                                 sizeof(dwVector2f),
                //                                                 0,
                //                                                 &boundaryWorldCharPtr,
                //                                                 count,
                //                                                 m_renderEngine));

                // std::cout << "Boundary point location in image space (pixels): "<< "<"
                // << m_freespaceBoundary.boundaryImagePoint[index].x << "," << m_freespaceBoundary.boundaryImagePoint[index].y << ">" 
                // << std::endl;
                // std::cout << "Boundary point location in car domain (meter):    "<< "<" 
                // << m_freespaceBoundary.boundaryWorldPoint[i].x << "," << m_freespaceBoundary.boundaryWorldPoint[i].y << ">" 
                // << std::endl;
                // myfile << "Writing this to a file.\n";
                // if(boundaryImagePointFileWriter){
                //     boundaryImagePointFileWriter << m_freespaceBoundary.boundaryImagePoint[index].x << "," << m_freespaceBoundary.boundaryImagePoint[index].y <<"\n";                    
                // }
                // if(boundaryWorldPointFileWriter){ 
                //     boundaryWorldPointFileWriter << m_freespaceBoundary.boundaryWorldPoint[i].x << "," << m_freespaceBoundary.boundaryWorldPoint[i].y << "\n";
                // }                
            }
            ++count;
        }
        // std::cout<<"====================================================================================================================="<<std::endl;

        // render detection roi
        dwRect roi{};
        CHECK_DW_ERROR(dwFreespaceDetector_getDetectionROI_new(0, &roi, m_freespaceDetector));
        dwRectf roif{static_cast<float32_t>(roi.x), static_cast<float32_t>(roi.y),
                    static_cast<float32_t>(roi.width), static_cast<float32_t>(roi.height)};
        CHECK_DW_ERROR(dwRenderEngine_setLineWidth(2, m_renderEngine));
        CHECK_DW_ERROR(dwRenderEngine_setColor(DW_RENDERER_COLOR_YELLOW, m_renderEngine));
        CHECK_DW_ERROR(dwRenderEngine_render(DW_RENDER_ENGINE_PRIMITIVE_TYPE_BOXES_2D, &roif,
                       sizeof(dwRect), 0, 1, m_renderEngine));
        // std::string roifStr =   std::to_string(roif.x) + ", " + 
        //                         std::to_string(roif.y) + ", " + 
        //                         std::to_string(roif.width) + ", " + 
        //                         std::to_string(roif.height);
                                               
        // const char *roifCharPtr = roifStr.c_str();
        // CHECK_DW_ERROR(dwRenderEngine_renderWithLabel( DW_RENDER_ENGINE_PRIMITIVE_TYPE_BOXES_2D, 
        //                                                 &roif, 
        //                                                 sizeof(dwRect), 
        //                                                 0,
        //                                                 roifCharPtr,
        //                                                 1, 
        //                                                 m_renderEngine));
        
        // std::cout << "ROI: "<< "x: " << roif.x << ", y: " << roif.y << ", width: " << roif.width << ", height: " << roif.height << std::endl;
        renderutils::renderFPS(m_renderEngine, getCurrentFPS());
        //#################################################################################################################################################
    }

    dwStatus actuate(int sp, int ang, int acc, int deltaAng){
        if(acc < 0){
            std::cout<<"Brake.."<<std::endl;
        }else {            
            std::cout<<"Move..."<<std::endl;            
        }
        int counter      = 1;
        int acceleration = acc;
        int init_angle   = ang;
        int delta_angle  = deltaAng;
        int s            = sp;
        // if (status != DW_FAILURE) {
            // while (true) {
                // send some data
                                
                
                s += acceleration;
                s = std::max(s, 0);
                init_angle += delta_angle;
                delta_angle -= 5;
                // if(init_angle < 180 || init_angle > 180) delta_angle *= -1;
                counter = (counter + 1) % 256;
                std::vector<uint8_t> data = buildMsgMABX(s, init_angle, delta_angle, 0, counter);
                //write speed data
                // if(speedFileWriter){
                //     speedFileWriter << s << "\n";                    
                // }
                // send data
                // std::this_thread::sleep_for(std::chrono::milliseconds(std::rand() % 500));
                const std::string sentDataStr(data.begin(), data.end());
                if ( ! pegasusSentSocket.send( sentDataStr ) )
                {
                    return DW_FAILURE;
                }
                // send data
                std::cout << "sent data: ";
                std::cout << sentDataStr << std::endl;
                
                std::cout << "=============================================================" << std::endl;
                
                std::string receiveDataStr;
                if ( ! pegasusReceiveSocket.recv( receiveDataStr ) )
                {
                    return DW_FAILURE;
                }


                // received data
                std::cout << "received data: ";
                std::cout << receiveDataStr << std::endl;
                std::cout << "=============================================================" << std::endl;
                // std::this_thread::sleep_for(std::chrono::milliseconds(std::rand() % 500));
                
            // }
        // }

        return DW_SUCCESS;
    }
    ///------------------------------------------------------------------------------
    /// Free up used memory here
    ///------------------------------------------------------------------------------
    void onRelease() override
    {
        pegasusReceiveSocket.close();
        pegasusSentSocket.close();

        
        if (m_imageRGBA)
        {
            dwImage_destroy(m_imageRGBA);
        }
        //##############################################///
        if (m_rigConfig)///
            dwRig_reset(m_rigConfig);///

        if (m_freespaceDetector)///
            dwFreespaceDetector_release(m_freespaceDetector);///

        if (m_openRoadNet)///
            dwOpenRoadNet_release(m_openRoadNet);///

        if(m_sal)///
            dwSAL_release(m_sal);///
        

        //###############################################///

        // Release detector
        CHECK_DW_ERROR(dwObjectDetector_release(m_driveNetDetector));
        // Release drivenet
        CHECK_DW_ERROR(dwDriveNet_release(m_driveNet));
        // Release object handles for detector and clusterer
        CHECK_DW_ERROR(dwObjectArray_destroy(&m_detectorOutput));
#ifdef VIBRANTE
        if (getArgument("input-type").compare("cameraCustom") == 0)
        {
            CHECK_DW_ERROR(dwSensor_stop(m_cameraMaster));
            dwSAL_releaseSensor(m_cameraMaster);
        }
#endif
        // Release camera
        m_camera.reset();

        // Release SDK
        CHECK_DW_ERROR(dwSAL_release(m_sal));

        if (m_renderEngine != DW_NULL_HANDLE)
        {
            CHECK_DW_ERROR(dwRenderEngine_release(m_renderEngine));
        }

        CHECK_DW_ERROR(dwVisualizationRelease(m_viz));
        CHECK_DW_ERROR(dwRelease(m_context));
        CHECK_DW_ERROR(dwLogger_release());

        CHECK_CUDA_ERROR(cudaStreamDestroy(m_cudaStream));
        objectFileWriter.close();
        boundaryImagePointFileWriter.close();
        boundaryWorldPointFileWriter.close();
        urgencyFileWriter.close();
        speedFileWriter.close();
        // CHECK_DW_ERROR(dwSocketConnection_release(socketConnectionWrite));
        // CHECK_DW_ERROR(dwSocketConnection_release(socketConnectionRead));
        // CHECK_DW_ERROR(dwSocketClient_release(socketClient));
    }

    ///------------------------------------------------------------------------------
    /// Reset detector
    ///------------------------------------------------------------------------------
    void onReset() override
    {
        dwFreespaceDetector_reset(m_freespaceDetector);
        dwOpenRoadNet_reset(m_openRoadNet);
    }

    ///------------------------------------------------------------------------------
    /// Change renderer properties when main rendering window is resized
    ///------------------------------------------------------------------------------
    void onResizeWindow(int width, int height) override
    {
        {
            CHECK_DW_ERROR(dwRenderEngine_reset(m_renderEngine));
            dwRectf rect;
            rect.width  = width;
            rect.height = height;
            rect.x      = 0;
            rect.y      = 0;
            CHECK_DW_ERROR(dwRenderEngine_setBounds(rect, m_renderEngine));
        }
    }
    
private:

    /// -----------------------------
    /// Initialize Logger and DriveWorks context
    /// -----------------------------
    void initializeDriveWorks(dwContextHandle_t& context) const
    {
        // initialize logger to print verbose message on console in color
        CHECK_DW_ERROR(dwLogger_initialize(getConsoleLoggerCallback(true)));
        CHECK_DW_ERROR(dwLogger_setLogLevel(DW_LOG_VERBOSE));

        // initialize SDK context, using data folder
        dwContextParameters sdkParams = {};
        sdkParams.dataPath            = DataPath::get_cstr();
        sdkParams.enableCudaTaskGraph = getArgument("useCudaGraph").compare("1") == 0;

        #ifdef VIBRANTE
        sdkParams.eglDisplay = getEGLDisplay();
        #endif

        CHECK_DW_ERROR(dwInitialize(&context, DW_VERSION, &sdkParams));
        std::cout << "exiting initializeDriveWorks()" << std::endl;
    }

    // void initializeActuation(dwContextHandle_t ctx){
    //     CHECK_DW_ERROR(dwInitialize(&ctx, DW_VERSION, nullptr));

    //     // auto pegasusIP      = getArgument("pegasusIP");

    //     // auto sendPort    = static_cast<uint16_t>(std::stoul(getArgument("sendPort")));
    //     // auto receivePort = static_cast<uint16_t>(std::stoul(getArgument("receivePort")));        
        
    //     // socketClient = dwSocketClientHandle_t{DW_NULL_HANDLE};
        
        
    //     // CHECK_DW_ERROR(dwSocketClient_initialize(&socketClient, 2, ctx));
    //     // do {
    //     //     status = dwSocketClient_connect(&socketConnectionRead, mabxip.c_str(), receivePort, 10000, socketClient);
    //     //     std::cout << "connecting socketConnectionRead.." << std::endl;  
    //     // } while (status == DW_TIME_OUT);          
    //     // do {
    //     //     status = dwSocketClient_connect(&socketConnectionWrite, mabxip.c_str(), sendPort, 10000, socketClient);
    //     //     std::cout << "connecting socketConnectionWrite.." << std::endl;  
    //     // } while (status == DW_TIME_OUT);

    //     try {

            

    //         std::string reply;

    //         try { 
    //         client_socket << "Test message.";
    //         client_socket >> reply;
    //         }
    //         catch ( SocketException& ) {}

    //         std::cout << "We received this response from the server:\n\"" << reply << "\"\n";;

    //     }
    //     catch ( SocketException& e ){
    //         std::cout << "Exception was caught:" << e.description() << "\n";
    //     }
    //     std::cout << "connected both sockets" << std::endl;  
    //     std::cout << "sockets created. exiting initializeActuation()" << std::endl;
    // }
    //------------------------------------------------------------------------------
    void processResults()
    {
        // log("Processing results...\n..");//added code
        // Clean up lists for each class
        for (uint32_t classIdx = 0U; classIdx < m_classLabels.size(); ++classIdx)
        {
            m_dnnLabelListPtr[classIdx].clear();
            m_dnnLabelList[classIdx].clear();
            m_dnnBoxList[classIdx].clear();
            m_urgencyList[classIdx].clear();
            m_isUrgencyValidList[classIdx].clear();
        }

        if(m_detectorOutput.count == 0){
            actuate(10, 0, 1, 0);
        }else {
            for (uint32_t objIdx = 0U; objIdx < m_detectorOutput.count; ++objIdx) {

                dwObjectCamera* obj = &static_cast<dwObjectCamera*>(m_detectorOutput.objects)[objIdx];
                dwObjectClass objectClass = obj->obstacle.objectClass;
                uint32_t classIdx = static_cast<uint32_t>(objectClass);

                m_dnnBoxList[classIdx].push_back(obj->box2D);

                std::string box_annotation = m_classLabels[classIdx];

                if (obj->isUrgencyValid)
                {
                    std::stringstream stream;
                    stream << std::setprecision(2) << obj->urgency << " 1/s";
                    // if(urgencyFileWriter){
                    //     urgencyFileWriter << obj->urgency << "\n";
                    // }
                    if(obj->urgency > 0.5f){
                        stream << "  DANGER!! Too close!";
                        // void actuate(int sp, int ang, int acc, int deltaAng)
                        actuate(0, 0, -5, 0);                    

                    }else {

                        actuate(10, 0, 1, 0);
                    }
                    box_annotation.append(", ");
                    box_annotation.append(stream.str());                

                }
                // This operation is safe because m_dnnLabelList is allocated using `reserve` at initialization
                // and it is not going to reallocate
                // std::cout << "Label List " << box_annotation << std::endl;//added code
                
                m_dnnLabelList[classIdx].push_back(box_annotation);
                m_dnnLabelListPtr[classIdx].push_back(m_dnnLabelList[classIdx].back().c_str());
                m_urgencyList[classIdx].push_back(obj->urgency);
                m_isUrgencyValidList[classIdx].push_back(obj->isUrgencyValid);
            }
        }
        
    }

    //------------------------------------------------------------------------------
    void getNextFrame(dwImageCUDA** nextFrameCUDA, dwImageGL** nextFrameGL)
    {
        dwImageHandle_t nextFrame = m_camera->readFrame();
        if (nextFrame == nullptr)
        {
            m_camera->resetCamera();
        }
        else
        {
            dwImage_getCUDA(nextFrameCUDA, nextFrame);
            //in freespace detection app, nextFrame is directly converted to frameGL but here first it is converted to m_imageRGBA
            CHECK_DW_ERROR(dwImage_copyConvert(m_imageRGBA, nextFrame, m_context));
            dwImageHandle_t frameGL = m_streamerCUDA2GL->post(m_imageRGBA);
            dwImage_getGL(nextFrameGL, frameGL);
        }
    }
    //#######################################################################################
    void runDetector(dwImageCUDA* frame)
     {
         // Run inference if the model is valid
         if (m_freespaceDetector)
         {
             CHECK_DW_ERROR(dwFreespaceDetector_detectBoundary(&m_freespaceBoundary, frame, m_freespaceDetector));
         }
     }
    
        
};

int main(int argc, const char** argv)
{
    // -------------------
    // define all arguments used by the application
    
    ProgramArguments args(argc, argv,
                        {
#ifdef VIBRANTE
                              ProgramArguments::Option_t("camera-type", "ar0231-rccb-bae-sf3324", "camera gmsl type (see sample_sensors_info for all available camera types on this platform)"),
                              ProgramArguments::Option_t("camera-group", "a", "input port"),
                              ProgramArguments::Option_t("camera-index", "0", "camera index within the camera-group 0-3"),
                              ProgramArguments::Option_t("slave", "0", "activate slave mode for Tegra B"),
                              ProgramArguments::Option_t("input-type", "video", "input type either video, camera or cameraCustom (using NvSIPL)"),
                              ProgramArguments::Option_t("cameraCustomString", "config-type=devBlock,configuration={MODE:master|INTERFACE:csi-a|CAMERA_NAME:SF3324|NUM:1},output-format=processed", "parameter string for cameraCustom"),
#endif
                              ProgramArguments::Option_t("pegasusIP", "192.168.50.2", "The server IP the client connects to"),
                              ProgramArguments::Option_t("mabxip", "192.168.50.1", "The mabx IP the pegasus connects to"),
                              ProgramArguments::Option_t("sendPort", "30000","The port the mabx will listen on / the client will send to"),
                              ProgramArguments::Option_t("receivePort", "51001","The port the pegasus will send on / the client will receive from"),
                              ProgramArguments::Option_t("dla", "0", "run inference on dla"),
                            //   ProgramArguments::Option_t("dlaEngineNo", "0", "dla engine to run DriveNet on if --dla=1"),modified below
                              ProgramArguments::Option_t("dlaEngineNo", "1", "dla engine to run DriveNet on if --dla=1"),
                            //   ProgramArguments::Option_t("video", (DataPath::get() + "/samples/raw/rccb.raw").c_str(), "path to video"),modified below
                              ProgramArguments::Option_t("video", "driveway.mp4", "path to video"),
                              ProgramArguments::Option_t("stopFrame", "0", "frame number indicating when to stop video"),
                            //   ProgramArguments::Option_t("precision", "fp32", "network precision. Possible options are \"int8\", \"fp16\" and \"fp32\"."),modified below
                              ProgramArguments::Option_t("precision", "fp16", "network precision. Possible options are \"int8\", \"fp16\" and \"fp32\"."),
                            //   ProgramArguments::Option_t("useCudaGraph", "0", "switch to use Cuda Graph for infer"),modified below
                              ProgramArguments::Option_t("useCudaGraph", "1", "switch to use Cuda Graph for infer"),                              
                            //   ProgramArguments::Option_t("enableUrgency", "0", "use temporal model to predict urgency"),
                              ProgramArguments::Option_t("enableUrgency", "1", "use temporal model to predict urgency"),///
                              ProgramArguments::Option_t("rig", (std::string{"rig_freespace.json"}).c_str()),
                              //ProgramArguments::Option_t("maxDistance", "50.0"),modified below
                              ProgramArguments::Option_t("maxDistance", "25.0"),///
                              ProgramArguments::Option_t("stateless", "0", "use stateless model. If not specified, use stateful model. This option must be used with enableUrgency=1")
                            //   ProgramArguments::Option_t("stateless", "1", "use stateless model. If not specified, use stateful model. This option must be used with enableUrgency=1")
                        },
                          "DriveNet sample which detects objects of multiple classes.");

    DriveNetSimpleApp app(args);
    app.initializeWindow("Object and FreeSpace Detection App", 1280, 800, args.enabled("offscreen"));
    app.setStopFrame(stoi(args.get("stopFrame")));
    

    return app.run();
}
