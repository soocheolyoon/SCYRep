// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <malloc.h>

#include <array>
#include <iostream>
#include <map>
#include <vector>

#include <fstream>
#include <string>

#include <k4a/k4a.h>
#include <k4abt.h>
#include <k4arecord/playback.h>


#include <BodyTrackingHelpers.h>
#include <Utilities.h>
#include <Window3dWrapper.h>

using namespace std;

void PrintUsage()
{
    printf("\nUSAGE: (k4abt_)simple_3d_viewer.exe SensorMode[NFOV_UNBINNED, WFOV_BINNED](optional) RuntimeMode[CPU](optional)\n");
    printf("  - SensorMode: \n");
    printf("      NFOV_UNBINNED (default) - Narraw Field of View Unbinned Mode [Resolution: 640x576; FOI: 75 degree x 65 degree]\n");
    printf("      WFOV_BINNED             - Wide Field of View Binned Mode [Resolution: 512x512; FOI: 120 degree x 120 degree]\n");
    printf("  - RuntimeMode: \n");
    printf("      CPU - Use the CPU only mode. It runs on machines without a GPU but it will be much slower\n");
    printf("e.g.   (k4abt_)simple_3d_viewer.exe WFOV_BINNED CPU\n");
    printf("e.g.   (k4abt_)simple_3d_viewer.exe CPU\n");
    printf("e.g.   (k4abt_)simple_3d_viewer.exe WFOV_BINNED\n");
}

void PrintAppUsage()
{
    printf("\n");
    printf(" Basic Navigation:\n\n");
    printf(" Rotate: Rotate the camera by moving the mouse while holding mouse left button\n");
    printf(" Pan: Translate the scene by holding Ctrl key and drag the scene with mouse left button\n");
    printf(" Zoom in/out: Move closer/farther away from the scene center by scrolling the mouse scroll wheel\n");
    printf(" Select Center: Center the scene based on a detected joint by right clicking the joint with mouse\n");
    printf("\n");
    printf(" Key Shortcuts\n\n");
    printf(" ESC: quit\n");
    printf(" h: help\n");
    printf(" b: body visualization mode\n");
    printf(" k: 3d window layout\n");
    printf("\n");
}

// Global State and Key Process Function
bool s_isRunning = true;
Visualization::Layout3d s_layoutMode = Visualization::Layout3d::OnlyMainView;
bool s_visualizeJointFrame = false;

int64_t ProcessKey(void* /*context*/, int key)
{
    // https://www.glfw.org/docs/latest/group__keys.html
    switch (key)
    {
        // Quit
    case GLFW_KEY_ESCAPE:
        s_isRunning = false;
        break;
    case GLFW_KEY_K:
        s_layoutMode = (Visualization::Layout3d)(((int)s_layoutMode + 1) % (int)Visualization::Layout3d::Count);
        break;
    case GLFW_KEY_B:
        s_visualizeJointFrame = !s_visualizeJointFrame;
        break;
    case GLFW_KEY_H:
        PrintAppUsage();
        break;
    }
    return 1;
}

int64_t CloseCallback(void* /*context*/)
{
    s_isRunning = false;
    return 1;
}

struct InputSettings
{
    k4a_depth_mode_t DepthCameraMode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    bool CpuOnlyMode = false;
};

bool ParseInputSettingsFromArg(int argc, char** argv, InputSettings& inputSettings)
{
    for (int i = 1; i < argc; i++)
    {
        std::string inputArg(argv[i]);
        if (inputArg == std::string("NFOV_UNBINNED"))
        {
            inputSettings.DepthCameraMode = K4A_DEPTH_MODE_NFOV_UNBINNED;
        }
        else if (inputArg == std::string("WFOV_BINNED"))
        {
            inputSettings.DepthCameraMode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
        }
        else if (inputArg == std::string("CPU"))
        {
            inputSettings.CpuOnlyMode = true;
        }
        else
        {
            return false;
        }
    }
    return true;
}


std::string TimeStamp()
{
	char str[32]{};

	time_t a = time(nullptr);

	struct tm time_info;

	// localtime_s, Microsoft version (returns zero on success, an error code on failure)
	if (localtime_s(&time_info, &a) == 0) strftime(str, sizeof(str), "%b-%d-%Y %H-%M-%S", &time_info);

	return str;
}

std::string TimeStamp_HMS()
{
	char str[32]{};

	time_t a = time(nullptr);

	struct tm time_info;

	// localtime_s, Microsoft version (returns zero on success, an error code on failure)
	if (localtime_s(&time_info, &a) == 0) strftime(str, sizeof(str), "%H-%M-%S", &time_info);

	return str;
}

int main(int argc, char** argv)
{
	//Output File Setup
	ofstream skeleton_out_csv;
	string current_time = TimeStamp();
	string csv_path = "C:\\Skeleton_Tracking_Data\\" + current_time + ".csv";

	//Open File Setup
	const char* filepath = "Test_for_read.mkv";


	skeleton_out_csv.open(csv_path);

	skeleton_out_csv << "Time,PELVIS, , , , , , , , SPINE_NAVAL, , , , , , , ,SPINE_CHEST, , , , , , , ,NECK, , , , , , , ,CLAVICLE_LEFT, , , , , , , ,SHOULDER_LEFT, , , , , , , ,ELBOW_LEFT, , , , , , , ,WRIST_LEFT, , , , , , , ,HAND_LEFT, , , , , , , ,HANDTIP_LEFT, , , , , , , ,THUMB_LEFT, , , , , , , ,CLAVICLE_RIGHT, , , , , , , ,SHOULDER_RIGHT, , , , , , , ,ELBOW_RIGHT, , , , , , , ,WRIST_RIGHT, , , , , , , ,HAND_RIGHT, , , , , , , ,HANDTIP_RIGHT, , , , , , , ,THUBM_RIGHT, , , , , , , ,HIP_LEFT, , , , , , , ,KNEE_LEFT, , , , , , , ,ANKLE_LEFT, , , , , , , ,FOOT_LEFT, , , , , , , ,HIP_RIGHT, , , , , , , ,KNEE_RIGHT, , , , , , , ,ANKLE_RIGHT, , , , , , , ,FOOT_RIGHT, , , , , , , ,HEAD, , , , , , , ,NOSE, , , , , , , ,EYE_LEFT, , , , , , , ,EAR_LEFT, , , , , , , ,EYE_RIGHT, , , , , , , ,EAR_RIGHT" << endl;
	skeleton_out_csv << "Time,";

	for (int i = 0; i < 32; i++)
		skeleton_out_csv << "x, y, z, ori_w, ori_x, ori_y, ori_z, conf, ";
	skeleton_out_csv << endl;

	k4a_playback_t file = NULL;
	
	k4a_result_t result = K4A_RESULT_SUCCEEDED;


	result = k4a_playback_open(filepath, &file);

	if (result != K4A_RESULT_SUCCEEDED)
	{
		printf("Failed to open file: %s\n", filepath);
		return 0;
	}

	k4a_record_configuration_t recordConfiguration;
	result = k4a_playback_get_record_configuration(file, &recordConfiguration);


	if (result != K4A_RESULT_SUCCEEDED)
	{
		printf("Failed to get record configuration for file: %s\n", filepath);
		return 0;
	}

	k4a_calibration_t fileCalibration;
	result = k4a_playback_get_calibration(file, &fileCalibration);

	if (result != K4A_RESULT_SUCCEEDED)
	{
		printf("Failed to get record calibration for file: %s\n", filepath);
		return 0;
	}

	k4a_capture_t fileCapture;
	k4a_stream_result_t stream_result = k4a_playback_get_next_capture(file, &fileCapture);

	if (stream_result == K4A_STREAM_RESULT_EOF)
	{
		printf("ERROR: Recording file is empty: %s\n", filepath);
		result = K4A_RESULT_FAILED;
		return 0;
	}
	else if (stream_result == K4A_STREAM_RESULT_FAILED)
	{
		printf("ERROR: Failed to read first capture from file: %s\n", filepath);
		result = K4A_RESULT_FAILED;
		return 0;
	}

	k4a_capture_release(fileCapture);

	

    int depthWidth = fileCalibration.depth_camera_calibration.resolution_width;
    int depthHeight = fileCalibration.depth_camera_calibration.resolution_height;

    // Create Body Tracker
    k4abt_tracker_t tracker = nullptr;
    k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
    //Original code ==> tracker_config.processing_mode = inputSettings.CpuOnlyMode ? K4ABT_TRACKER_PROCESSING_MODE_CPU : K4ABT_TRACKER_PROCESSING_MODE_GPU;
	tracker_config.processing_mode = K4ABT_TRACKER_PROCESSING_MODE_GPU;

	VERIFY(k4abt_tracker_create(&fileCalibration, tracker_config, &tracker), "Body tracker initialization failed!");
    // Initialize the 3d window controller
    Window3dWrapper window3d;
    window3d.Create("3D Visualization", fileCalibration);
    window3d.SetCloseCallback(CloseCallback);
    window3d.SetKeyCallback(ProcessKey);

    while (1)
    {
		stream_result = k4a_playback_get_next_capture(file, &fileCapture);

		if (stream_result == K4A_STREAM_RESULT_EOF)
		{
			break;
		}

        if (stream_result == K4A_STREAM_RESULT_SUCCEEDED)
        {

            k4a_wait_result_t queueCaptureResult = k4abt_tracker_enqueue_capture(tracker, fileCapture, 0);

            // Release the sensor capture once it is no longer needed.
            k4a_capture_release(fileCapture);

            if (queueCaptureResult == K4A_WAIT_RESULT_FAILED)
            {
                std::cout << "Error! Add capture to tracker process queue failed!" << std::endl;
                break;
            }
        }
        else if (stream_result != K4A_WAIT_RESULT_TIMEOUT)
        {
            std::cout << "Get depth capture returned error: " << stream_result << std::endl;
            break;
        }

        // Pop Result from Body Tracker
        k4abt_frame_t bodyFrame = nullptr;
        k4a_wait_result_t popFrameResult = k4abt_tracker_pop_result(tracker, &bodyFrame, 0); // timeout_in_ms is set to 0
        if (popFrameResult == K4A_WAIT_RESULT_SUCCEEDED)
        {
            /************* Successfully get a body tracking result, process the result here ***************/

            // Obtain original capture that generates the body tracking result
            k4a_capture_t originalCapture = k4abt_frame_get_capture(bodyFrame);
            k4a_image_t depthImage = k4a_capture_get_depth_image(originalCapture);

            std::vector<Color> pointCloudColors(depthWidth * depthHeight, { 1.f, 1.f, 1.f, 1.f });

            // Read body index map and assign colors
            k4a_image_t bodyIndexMap = k4abt_frame_get_body_index_map(bodyFrame);
            const uint8_t* bodyIndexMapBuffer = k4a_image_get_buffer(bodyIndexMap);
            for (int i = 0; i < depthWidth * depthHeight; i++)
            {
                uint8_t bodyIndex = bodyIndexMapBuffer[i];
                if (bodyIndex != K4ABT_BODY_INDEX_MAP_BACKGROUND)
                {
                    uint32_t bodyId = k4abt_frame_get_body_id(bodyFrame, bodyIndex);
                    pointCloudColors[i] = g_bodyColors[bodyId % g_bodyColors.size()];
                }
            }
            k4a_image_release(bodyIndexMap);

            // Visualize point cloud
            window3d.UpdatePointClouds(depthImage, pointCloudColors);

            // Visualize the skeleton data
            window3d.CleanJointsAndBones();
            uint32_t numBodies = k4abt_frame_get_num_bodies(bodyFrame);
            for (uint32_t i = 0; i < numBodies; i++)
            {
                k4abt_body_t body;
                VERIFY(k4abt_frame_get_body_skeleton(bodyFrame, i, &body.skeleton), "Get skeleton from body frame failed!");
                body.id = k4abt_frame_get_body_id(bodyFrame, i);

                // Assign the correct color based on the body id
                Color color = g_bodyColors[body.id % g_bodyColors.size()];
                color.a = 0.4f;
                Color lowConfidenceColor = color;
                lowConfidenceColor.a = 0.1f;

				// Get current time
				
				skeleton_out_csv << k4a_image_get_device_timestamp_usec(depthImage) << ",";

                // Visualize joints
                for (int joint = 0; joint < static_cast<int>(K4ABT_JOINT_COUNT); joint++)
                {
                    if (body.skeleton.joints[joint].confidence_level >= K4ABT_JOINT_CONFIDENCE_LOW)
                    {
                        const k4a_float3_t& jointPosition = body.skeleton.joints[joint].position;
                        const k4a_quaternion_t& jointOrientation = body.skeleton.joints[joint].orientation;

						skeleton_out_csv << jointPosition.v[0] << "," << jointPosition.v[1] << ","<< jointPosition.v[2] << ", " ;
						skeleton_out_csv << jointOrientation.v[0] << "," << jointOrientation.v[1] << "," << jointOrientation.v[2] << "," << jointOrientation.v[3] << ",";
						skeleton_out_csv << body.skeleton.joints[joint].confidence_level << ",";

                        window3d.AddJoint(
                            jointPosition,
                            jointOrientation,
                            body.skeleton.joints[joint].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM ? color : lowConfidenceColor);
                    }
                }

				skeleton_out_csv << endl;


                // Visualize bones
                for (size_t boneIdx = 0; boneIdx < g_boneList.size(); boneIdx++)
                {
                    k4abt_joint_id_t joint1 = g_boneList[boneIdx].first;
                    k4abt_joint_id_t joint2 = g_boneList[boneIdx].second;

                    if (body.skeleton.joints[joint1].confidence_level >= K4ABT_JOINT_CONFIDENCE_LOW &&
                        body.skeleton.joints[joint2].confidence_level >= K4ABT_JOINT_CONFIDENCE_LOW)
                    {
                        bool confidentBone = body.skeleton.joints[joint1].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM &&
                                             body.skeleton.joints[joint2].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM;
                        const k4a_float3_t& joint1Position = body.skeleton.joints[joint1].position;
                        const k4a_float3_t& joint2Position = body.skeleton.joints[joint2].position;

                        window3d.AddBone(joint1Position, joint2Position, confidentBone ? color : lowConfidenceColor);
                    }
                }
            }

            k4a_capture_release(originalCapture);
            k4a_image_release(depthImage);
            k4abt_frame_release(bodyFrame);
        }

        window3d.SetLayout3d(s_layoutMode);
        window3d.SetJointFrameVisualization(s_visualizeJointFrame);
        window3d.Render();
    }

    std::cout << "Finished body tracking processing!" << std::endl;

    window3d.Delete();
    k4abt_tracker_shutdown(tracker);
    k4abt_tracker_destroy(tracker);

	skeleton_out_csv.close();

	k4a_playback_close(file);


    return 0;
}
