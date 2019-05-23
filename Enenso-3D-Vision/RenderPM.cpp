/////////////////////////////////////////////////////////////////////////
// RenderPM_1.cpp
// Purpose: Uses an Ensenso 3D camera to obtain a colored point
//			cloud of an object's surface. Maps the XYZ values 
//			of the points in the point cloud with their
//			corresponding RGBA values to extract the contour
//			of the largest black region on the object's surface.
//
// Author: Raisaat Rashid
// Date: March 13, 2019
/////////////////////////////////////////////////////////////////////////

// Required libraries
#include "nxLib.h"
#include <iostream>
#include <iomanip>
#include <cstdio>
#include <stdio.h>
#include <vector>
#include <algorithm>
#include <math.h>

#include <pcl/common/common.h>
#include <pcl/console/print.h>
#include <pcl/io/ensenso_grabber.h>
#include <pcl/console/time.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/image_viewer.h>

#include <opencv2/core/core.hpp>
#include "opencv2/opencv.hpp"
#include "cv.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// Namespaces
using namespace std;
using namespace cv;

// Structure to hold the XYZ values of each point in the point cloud
struct XYZ
{
	float x, y, z;
};

// Structure to hold the RGBA values of each point in the point cloud
struct RGBA
{
	char r, g, b, a;
};

int main(void) 
{
	try 
	{
		// Initialize NxLib and enumerate the cameras
		nxLibInitialize(true);
		
		// Open TCP port to inspect tree with NxTreeEdit
		nxLibOpenTcpPort(24001);

		// Reference to cameras node (use reference by path here)
		NxLibItem Cams = NxLibItem("/Cameras/BySerialNo");

		// Display the number of cameras
		cout << "Number of cameras: " << Cams.count() << endl;
		
		// Print the serial number of each camera in the tree
		int w = 12;
		cout << setiosflags(ios::left) << setw(w) << "Serial No" << setw(w) << "Model" << endl;
		for (int n = 0; n < Cams.count(); ++n)
			cout << setw(w) << Cams[n][itmSerialNumber].asString() << setw(w) << Cams[n][itmModelName].asString() << endl;

		// If the no. of cameras connected is less than 2, i.e. the ensenso camera and/or
		// the RGB camera are/is not connected, print an error message
		if (Cams.count() < 2) throw "There must be at least 2 cameras.\n";

		// Reference to the first two cameras in the node by serial no.
		NxLibItem root; // root of the tree
		NxLibItem camera0 = root[itmCameras][itmBySerialNo][0]; // Ensenso camera
		NxLibItem camera1 = root[itmCameras][itmBySerialNo][1]; // RGB camera

		// Open both cameras, specifying explicitly which ones to open; by default all available would be opened
		NxLibCommand open(cmdOpen);
		string camString;
		camString = "[\""+camera0[itmSerialNumber].asString()+"\",\""+camera1[itmSerialNumber].asString()+"\"]";
		open.parameters()[itmCameras].setJson(camString,true);
		cout << "Opening the Ensenso camera and the RGB camera.\n";
		open.execute();

		// A file named 'updated_params.json' containing the Ensenso camera's settings was saved from NxView GUI that will now be loaded
		// First read the json file as a normal text file into a string variable
		cout << "Reading the camera parameters.\n";
		ifstream file1("updated_param.json");
		if (file1.is_open() && file1.rdbuf()) 
		{
			// Use the stringstream class to read text file
			stringstream buffer;
			buffer << file1.rdbuf();
			string const& fileContent = buffer.str(); // The entire json representation of the parameter set is in 'fileContent' as a string
			
			// In order to detect if the file contains the parameters only, or the entire camera node, it is simply written into
			// a temporary NxLib node "/tmp" and checked if it contains a subitem 'Parameters'; if it does, only this 
			// subitem is used to rewrite the camera's parameter tree, otherwise it is assumed that the file contains the parameter subtree only
			NxLibItem tmp("/tmp");
			tmp.setJson(fileContent); // Parse json file content into a temporary item
			if (tmp[itmParameters].exists()) // If there is a Parameters subnode, the file contained the full camera node
			{
				camera0[itmParameters].setJson(tmp[itmParameters].asJson(), true); // Use the parameters subtree to overwrite the camera's parameter subtree
			}
			else // If the file contained the parameters node only
			{
				camera0[itmParameters].setJson(tmp.asJson(), true); // Use the entire content of the temporary node to rewrite the camera's parameters node 
			}
		} 
		else // If the file could not be read, print an error message
			cout << "The parameters file could not be read.\n";

		// Capture an image from all open cameras
		cout << "Capturing images.\n";
		NxLibCommand (cmdCapture).execute();

		// Compute disparity map (this also computes the rectfied images)
		cout << "Computing disparity map.\n";
		NxLibCommand (cmdComputeDisparityMap).execute();

		// Compute XYZ data for each pixel
		cout << "Computing point map.\n";
		NxLibCommand (cmdComputePointMap).execute();

		// Compute map using data from the RGB camera (data from the Ensenso camera is used by default)
		compRenderPointMap.parameters()[itmCamera].set(camera1[itmSerialNumber].asString());
		compRenderPointMap.parameters()[itmNear].set(50);
		cout << "Computing render point map\n";
		compRenderPointMap.execute();

		vector<XYZ> renderPointMap;	// Vector to hold the XYZ values of all the points in the point cloud
		vector<RGBA> rgbData;		// Vector to hold the RGBA values of all the ponts in the point cloud
		
		int width,	// Variable to hold the width of the point cloud 
			height,	// Variable to hold the height of the point cloud

		root[itmImages][itmRenderPointMap].getBinaryDataInfo(&width, &height, 0,0,0,0); // Get the width and height of the point cloud
		root[itmImages][itmRenderPointMap].getBinaryData(renderPointMap, 0); // Get the XYZ data of the point cloud
		root[itmImages][itmRenderPointMapTexture].getBinaryData(rgbData, 0); // Get the RGBA data of the point cloud
		
		Mat coordinateMatrix = Mat(height, width, CV_64FC3);	// Matrix to hold the XYZ values of all the points
		Mat colorMatrix = Mat(height, width, CV_8UC4);		// Matrix to hold the RGBA values of all the points
		
		// Transfer the XYZ data from the vector 'renderPointMap' to the matrix 'coordinateMatrix'
		cout << "Transferring xyz data to a matrix.\n";
		int counter = 0; // Counter variable to go through the vector 'renderPointMap'
		for (int i = 0; i < coordinateMatrix.rows; i++)
		{
			for (int j = 0; j < coordinateMatrix.cols; j++)
			{
				// Transfer the XYZ values of the current point in the vector 'renderPointMap' to an element in the matrix 'coordinateMatrix'
				coordinateMatrix.at<Vec3f>(i,j) = Vec3f(renderPointMap[counter].x, renderPointMap[counter].y, renderPointMap[counter].z);
				
				// Increment the counter to go to the next point in the vector 'renderPointMap'
				counter++;
			}
		}

		// Transfer the RGBA data from the vector 'rgbData' to the matrix 'colorMatrix'
		cout << "Transferring rgb data to a matrix.\n";
		counter = 0; // Counter variable to go through the vector 'rgbData'
		for (int i = 0; i < colorMatrix.rows; i++)
		{
			for (int j = 0; j < colorMatrix.cols; j++)
			{
				// Transfer the RGBA values of the current element in the vector 'rgbData' to an element in the matrix 'colorMatrix'
				colorMatrix.at<Vec4b>(i,j) = Vec4b(rgbData[counter].r, rgbData[counter].g, rgbData[counter].b, rgbData[counter].a);
				
				// Increment the counter to go to the next element of the vector 'rgbData'
				counter++;
			}
		}
		
		// Find the contours

		cout << "Finding the contours.\n";
	    
		Mat processedImage; // Matrix to be used for image processing
		
		// Convert the colored image to a grayscale image
	    cvtColor(colorMatrix, processedImage, CV_RGB2GRAY);

		// Save the grayscale image
		imwrite("gray_image.jpg", processedImage);

	    // Blur to smoothen the edges of the object outlines in the image
	    blur(processedImage, processedImage, Size(3,3));

	    // Apply thresholding to separate the dark and light regions in the image in order to produce a binary image
	    threshold(processedImage, processedImage, 60, 255, CV_THRESH_BINARY); // Lower the value of 40 to get darker areas (0 is black, 255 is white)

	    // Save the threshold image
		imwrite("threshold_image.jpg", processedImage);
	    
	    // Fill the holes in the image with white color
		floodFill(processedImage, Point(0,0), Scalar(255));

	    // Note: This part is only for dark objects against light background (delete this part for light objects against dark background)
	    // Convert the black regions to white and vice-versa
	    bitwise_not(processedImage, processedImage);

		// Save the inverted image
	   	imwrite("threshold_image_inverted.jpg", processedImage);

	    // Perform Morphological Closing on the image
	    // Closing is Dilation followed by Erosion. Closing is used to close small holes and small black points on the object
	    int morph_size = 2;
	    Mat element = getStructuringElement( MORPH_ELLIPSE, Size( 2*morph_size + 1, 2*morph_size+1 ), Point( morph_size, morph_size ) ); // To create a rectangular structural element, replace MORPH_ELLIPSE with MORPH_RECT
	    morphologyEx( processedImage, processedImage, MORPH_CLOSE, element, Point(-1, -1), 1, BORDER_REPLICATE);
	    
	    // Save the morphed image
		imwrite("morph_closing_image.jpg", processedImage);

	    // Find the contours
	    vector<vector<Point>> contours;
	    findContours(processedImage, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

	    // Find the largest contour
		cout << "Finding the largest contour.\n";
		double largestArea = 0; // Variable to hold the area of the largest contour
		int largestIndex = 0; // Variable to hold the index of the element in the vector 'contours' which contains the largest contour
	    for(int i = 0; i < contours.size(); i++)
	    {
	        double a = contourArea(contours[i],false); // Find the area of the current contour
	        
			if (a > largestArea) // If the current contour's area is larger than the largest contour area saved
	        {
	            largestArea = a; // Save the current contour's area
	            largestIndex = i; // Save the index of the current contour
	        }
	    }

		// Assign the largest contour to a vector
		vector<Point> largestContour = contours.at(largestIndex);

	    // Save the contour points in a text file
	    cout << "Saving the contour points in the file contour_points.txt.\n";
	    ofstream outfile;
  		outfile.open ("contour_points.txt");
	    for (int i = 0; i < largestContour.size(); i++) // Loop through all the points of the vector 'largestContour'
	    {
		    Point coordinate = largestContour.at(i); // Get the current point in the vector 'largestContour'

			if (!isnan(coordinateMatrix.at<Vec3f>(coordinate.y, coordinate.x)[0])) // If the current point is not a 'NaN'
			{
				outfile << (coordinateMatrix.at<Vec3f>(coordinate.y, coordinate.x)[0]) << " "; // Save the X value of the point
				outfile << (coordinateMatrix.at<Vec3f>(coordinate.y, coordinate.x)[1]) << " "; // Save the Y value of the point
				outfile << (coordinateMatrix.at<Vec3f>(coordinate.y, coordinate.x)[2]) << ";\n"; // Save the Z value of the point and go to the next line
			}
		}
		outfile.close(); // close the file contour_points.txt
	    
	    // Draw the largest contour on the original image
		vector<vector<Point>> contours_2; // A new vector to hold the largest contour only
		contours_2.push_back(contours[largestIndex]); // Push the largest contour to the vector 'contours_2'
	    drawContours(colorMatrix, contours_2, -1, (230,200,230), 2); // Draw the largest contour

		// Save the image in the current working directory
		cout << "Saving the contoured image.\n";
	    imwrite("contoured_wound.jpg", colorMatrix);

		// Save the RenderPointMap structure as an image in the current working directory
		NxLibCommand saveImage(cmdSaveImage);
		saveImage.parameters()[itmFilename] = "RenderPointMap.png";
		saveImage.parameters()[itmNode] = root[itmImages][itmRenderPointMap].path;
		cout << "Saving the rendered point map image.\n";
		saveImage.execute();

		// Close both the cameras
		NxLibCommand close(cmdClose);
		close.parameters()[itmCameras].setJson(camString,true);
		cout << "Closing the Ensenso camera and the RGB camera.\n";
		close.execute();

		// Close NxLib
		cout << "Closing NxLib.\n";
		nxLibCloseTcpPort();
		nxLibFinalize();
	} 
	catch (NxLibException ex) 
	{
		cout << ex.getItemPath() << endl;
		cout << ex.getErrorText() << endl;
	} 
	catch (char const* e) 
	{
		cout << e << endl;
	}	
	printf("Press enter to quit.\n");
	std::getchar();
return 0;
}