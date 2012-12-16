/* KinectViewer.cpp
*
* Copyright 2010-2011 Johannes Kepler Universität Linz,
* Institut für Wissensbasierte Mathematische Systeme.
*
* This file is part of pureImage.
*
* pureImage is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License, version 3,
* as published by the Free Software Foundation.
*
* pureImage is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty
* of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
* See the GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with pureImage. If not, see <http://www.gnu.org/licenses/>.
*/

#define NOMINMAX

#include <cstdio>

#if 1
#ifndef __GNUC__
#include <conio.h>
#else
#include "gcc_conio.h"
#endif
#endif

#include <cmath>

#include <map>

#include <iostream>

#include <boost/shared_array.hpp>

#include <Poco/Path.h>
#include <Poco/Timestamp.h>
#include <Poco/DateTimeFormatter.h>

#include <PureImage.hpp>
#include <BaseApplication.hpp>
#include <Arguments/Arguments.hpp>
#include <Arguments/IntValue.hpp>
#include <Arguments/StringSelection.hpp>
#include <Arguments/ShortMatrix.hpp>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>


// Several methods how to convert Kinect depth to real world coordinates
// are suggested throughout the web.

// The most common method, mentioned for instance at
// http://nicolas.burrus.name/index.php/Research/KinectCalibration,
// is to take the inverse of a linear function of the depth.
double RawDepthToMeters_InvLinear (int depthValue) {
    const double k1 = 3.3309495161;
    const double k2 = -0.0030711016;

    if (depthValue < 2047) {
        return (1.0 / (k1 + k2 * (double) depthValue));
    }

    return 0.0;
}

// In http://tinyurl.com/6fxflsp, Stephane Magnenat mentions that using
// tan() is slightly better than the inverse.
double RawDepthToMeters_Tan1 (int depthValue) {
    const double k1 = 1.1863;
    const double k2 = 2842.5;
    const double k3 = 0.1236;

    if (depthValue < 2047) {
        return (k3 * tan (k1 + (double) depthValue / k2));
    }

    return 0.0;
}

//In http://vvvv.org/forum/the-kinect-thread, user marf reports this formula:
double RawDepthToMeters_Tan2 (int depthValue) {
    const double k1 = 0.5;
    const double k2 = 1024.;
    const double k3 = 33.825;
    const double k4 = 5.7;

    if (depthValue < 2047) {
        return 0.01 * (k3 * tan (k1 + (double) depthValue / k2) + k4);
    }

    return 0.0;
}

double RawDepthToMeters (int depthValue) {
    return RawDepthToMeters_Tan1 (depthValue);
}


//The following functionis mentioned at http://graphics.stanford.edu/~mdfisher/Kinect.html
pcl::PointXYZ DepthToWorld (int x, int y, int depthValue) {
    const double fx_d = 1.0 / 5.9421434211923247e+02;
    const double fy_d = 1.0 / 5.9104053696870778e+02;
    const double cx_d = 3.3930780975300314e+02;
    const double cy_d = 2.4273913761751615e+02;

    pcl::PointXYZ result;
    double depth = RawDepthToMeters (depthValue);
    result.x = (float) ( (x - cx_d) * depth * fx_d);
    result.y = (float) ( (y - cy_d) * depth * fy_d);
    result.z = (float) (depth);
    return result;
}


uint32_t red (uint32_t hex) {
    return (hex >> 16) & 0xFF;
}

uint32_t green (uint32_t hex) {
    return (hex >> 8) & 0xFF;
}

uint32_t blue (uint32_t hex) {
    return hex & 0xFF;
}

/**
 * Builds a color ramp which ramps between n base colors given as a map.
 * Color values are given in hexadecimal form, i.e. red is 0xFF0000, green is 0x00FF00, etc.
 * Inspired by http://paulbourke.net/texture_colour/colourramp/.
*/
struct ColorRamp {
    ColorRamp (const std::map<size_t, uint32_t>& colors)
        : _colors (colors)
    {}

    uint32_t GetColor (size_t v) {

        std::map<size_t, uint32_t>::const_iterator it = _colors.find (v);

        // If key v is contained in the map, returns the corresponding color.
        if (it != _colors.end()) {
            return it->second;
        }

        std::map<size_t, uint32_t>::const_iterator upr = _colors.upper_bound (v);

        // If v is smaller than the smallest, or larger than the largest key,
        // returns the color of the smallest or largest element, resp.
        if (upr == _colors.begin()) {
            return upr->second;
        }

        if (upr == _colors.end()) {
            return (--upr)->second;
        }

        // Otherwise, interpolates between the colors of the smaller and larger elements.
        std::map<size_t, uint32_t>::const_iterator lwr = upr;
        --lwr;

        double ratio = ( (double) v - (double) lwr->first) / ( (double) upr->first - (double) lwr->first);

        double cr = (1. - ratio) * red (lwr->second)   + ratio * red (upr->second);
        double cg = (1. - ratio) * green (lwr->second) + ratio * green (upr->second);
        double cb = (1. - ratio) * blue (lwr->second)  + ratio * blue (upr->second);

        uint32_t c = ( (uint32_t) floor (cr) << 16)
                     | ( (uint32_t) floor (cg) << 8) | ( (uint32_t) floor (cb));

        return c;
    }

private:
    std::map<size_t, uint32_t> _colors;
};


/// Loads and registers plugin libraries
bool LoadPlugins (pI::BaseApplication& app) {

    std::string baseDir = app.FindPIBaseDir();

    if (baseDir.empty()) {
        return false;
    }

    Poco::Path pluginDir (baseDir);

#ifdef _DEBUG
    pluginDir.pushDirectory ("DistDebug");
#else
    pluginDir.pushDirectory ("DistRelease");
#endif

    app.LoadPluginLibrary (pluginDir.toString().c_str(), "CImg_pIns");
    app.LoadPluginLibrary (pluginDir.toString().c_str(), "FLLL_pIns");
    app.LoadPluginLibrary (pluginDir.toString().c_str(), "OpenCV_pIns");
    app.LoadPluginLibrary (pluginDir.toString().c_str(), "Freenect_pIns");

    app.RegisterPlugins();
    return true;
}


int main (int argc, char** argv) {

    const bool USE_OPENCV_DISPLAY = false; // Use OpenCV or CImg for display?

    pI::BaseApplication app;
    pI::Runtime runtime (app.GetRuntime());
    LoadPlugins (app);

    // Creates and initializes all the necessary plugins.
    const int DeviceIndex = 0;
    const bool ShowRGB = true;

    // Freenect/GetVideoAndDepth returns both the RGB image and the depth map captured by a Kinect.
    boost::shared_ptr<pI::pIn> getImageAndDepth
        = app.SpawnAndInitialize (runtime,
                                  "Freenect/GetVideoAndDepth",
                                  DeviceIndex,
                                  ShowRGB ? 0 : 2,   // Video format: FREENECT_VIDEO_RGB or FREENECT_VIDEO_IR_8BIT
                                  0                  // Depth format: FREENECT_DEPTH_11BIT
                                 );
    pI::Arguments getImageAndDepth_in = getImageAndDepth->GetInputSignature(),
                  getImageAndDepth_out = getImageAndDepth->GetOutputSignature();

    // Freenect/SetTiltAndLED sets the tilt angle and LED status of a Kinect.
    boost::shared_ptr<pI::pIn> setTiltAndLED = app.SpawnAndInitialize (runtime, "Freenect/SetTiltAndLED", DeviceIndex);
    pI::Arguments setTiltAndLED_in = setTiltAndLED->GetInputSignature(),
                  setTiltAndLED_out = setTiltAndLED->GetOutputSignature();

    // Color/ApplyRGBPalette applies an arbitrary RGB palette onto a matrix
    boost::shared_ptr<pI::pIn> applyRGB (runtime.SpawnPlugin ("pIn/Color/ApplyRGBPalette"));
    pI::Arguments applyRGB_params = applyRGB->GetParameterSignature();

    pI::RGBPalette rgb (applyRGB_params[0]);
    rgb.SetCount (2048);

    std::map<size_t, uint32_t> freenect_glview_colors; // Coloring as used in Freenect's glview example
    freenect_glview_colors[0]    = (0xFFFFFF); // white
    freenect_glview_colors[256]  = (0xFF0000); // red
    freenect_glview_colors[512]  = (0xFFFF00); // yellow
    freenect_glview_colors[768]  = (0x00FF00); // green
    freenect_glview_colors[1024] = (0x00FFFF); // cyan
    freenect_glview_colors[1280] = (0x0000FF); // blue
    freenect_glview_colors[1536] = (0x000000); // black

    std::map<size_t, uint32_t> terrain; // Alternative coloring; names taken from http://en.wikipedia.org/wiki/List_of_colors
    terrain[0]     = (0xFFFFFF); // White
    terrain[256]   = (0x964B00); // Brown (traditional)
    terrain[512]   = (0xFBEC5D); // Maize
    terrain[768]   = (0x66FF00); // Bright green
    terrain[1024]  = (0x014421); // Forest green (traditional)
    terrain[1025]  = (0x87CEFA); // Light sky blue
    terrain[1280]  = (0x00008B); // Dark blue
    terrain[2047]  = (0x000000); // Black
    terrain[2048]  = (0x696969); // Dim gray

    ColorRamp ramp (terrain);

    for (int i = 0; i < 2048; i++) {
        float v = powf ( (i / 2048.), 3);
        uint16_t gamma = 9216 * v;

        uint32_t c = ramp.GetColor (gamma);
        rgb.SetR (i, red (c));
        rgb.SetG (i, green (c));
        rgb.SetB (i, blue (c));
    }

    applyRGB->Initialize (applyRGB_params);

    pI::Arguments applyRGB_in = applyRGB->GetInputSignature(),
                  applyRGB_out = applyRGB->GetOutputSignature();

    // OpenCV/Draw/Rectangle draws the inner rectangle.
    boost::shared_ptr<pI::pIn> drawRectImage = app.SpawnAndInitialize (runtime, "OpenCV/Draw/Rectangle");
    pI::Arguments drawRectImage_in = drawRectImage->GetInputSignature(),
                  drawRectImage_out = drawRectImage->GetOutputSignature();

    boost::shared_ptr<pI::pIn> drawRectDepth = app.SpawnAndInitialize (runtime, "OpenCV/Draw/Rectangle");
    pI::Arguments drawRectDepth_in = drawRectDepth->GetInputSignature(),
                  drawRectDepth_out = drawRectDepth->GetOutputSignature();

    // OpenCV/Draw/PutText draws a text string into an image.
    boost::shared_ptr<pI::pIn> puttext
    = app.SpawnAndInitialize (runtime,
                              "OpenCV/Draw/PutText",
                              0,                // font type: CV_FONT_HERSHEY_SIMPLEX
                              pI_FALSE,         // render font italic: no
                              0.75, 0.75,       // horizontal and vertical scale
                              0.0,              // shear: no shear
                              1,                // thickness of the text strokes: 1
                              2                 // type of the line: CV_AA
                             );
    pI::Arguments puttext_in = puttext->GetInputSignature(),
                  puttext_out = puttext->GetOutputSignature();

    // OpenCV/IO/SaveImage saves images to disk.
    boost::shared_ptr<pI::pIn> saveImage (runtime.SpawnPlugin ("OpenCV/IO/SaveImage"));
    saveImage->Initialize (saveImage->GetParameterSignature());
    pI::Arguments saveImage_in = saveImage->GetInputSignature(),
                  saveImage_out = saveImage->GetOutputSignature();

    // OpenCV/IO/ShowImage or CImg/Display displays the image and the depth map.
    boost::shared_ptr<pI::pIn> displayImage, displayDepth;

    if (USE_OPENCV_DISPLAY) {
        displayImage = app.SpawnAndInitialize (runtime, "OpenCV/IO/ShowImage",
                                               "RGB image window", false, 640, 480, 0, 0);
        displayDepth = app.SpawnAndInitialize (runtime, "OpenCV/IO/ShowImage",
                                               "Depth map window", false, 640, 480, 650, 0);
    } else {
        displayImage   = app.SpawnAndInitialize (runtime, "CImg/Display", pI_TRUE);
        displayDepth = app.SpawnAndInitialize (runtime, "CImg/Display", pI_TRUE);
    }

    pI::Arguments displayImage_in = displayImage->GetInputSignature(),
                  displayImage_out = displayImage->GetOutputSignature();

    pI::Arguments displayDepth_in = displayDepth->GetInputSignature(),
                  displayDepth_out = displayDepth->GetOutputSignature();


    // Now that all plugins are initialized, connect them.

    drawRectImage_in[0] = getImageAndDepth_out[0];
    pI::IntValue (drawRectImage_in[1]).SetData (310);
    pI::IntValue (drawRectImage_in[2]).SetData (230);
    pI::IntValue (drawRectImage_in[3]).SetData (330);
    pI::IntValue (drawRectImage_in[4]).SetData (250);
    pI::IntValue (drawRectImage_in[5]).SetData (255);
    pI::IntValue (drawRectImage_in[6]).SetData (255);
    pI::IntValue (drawRectImage_in[7]).SetData (255);
    pI::IntValue (drawRectImage_in[8]).SetData (1);
    pI::StringSelection (drawRectImage_in[9]).SetIndex (0);
    pI::IntValue (drawRectImage_in[10]).SetData (0);

    displayImage_in[0] = drawRectImage_in[0];

    pI::StringValue fileName (saveImage_in[0]);
    saveImage_in[1] = getImageAndDepth_out[0];

    applyRGB_in[0] =  getImageAndDepth_out[1];
    drawRectDepth_in[0] = applyRGB_out[0];
    pI::IntValue (drawRectDepth_in[1]).SetData (310);
    pI::IntValue (drawRectDepth_in[2]).SetData (230);
    pI::IntValue (drawRectDepth_in[3]).SetData (330);
    pI::IntValue (drawRectDepth_in[4]).SetData (250);
    pI::IntValue (drawRectDepth_in[5]).SetData (255);
    pI::IntValue (drawRectDepth_in[6]).SetData (255);
    pI::IntValue (drawRectDepth_in[7]).SetData (255);
    pI::IntValue (drawRectDepth_in[8]).SetData (1);
    pI::StringSelection (drawRectDepth_in[9]).SetIndex (0);
    pI::IntValue (drawRectDepth_in[10]).SetData (0);
    displayDepth_in[0] = drawRectDepth_in[0];

    pI::StringValue puttext_text (puttext_in[1]);
    pI::IntValue puttext_x (puttext_in[2]);
    pI::IntValue puttext_y (puttext_in[3]);
    pI::IntValue puttext_R (puttext_in[4]);
    pI::IntValue puttext_G (puttext_in[5]);
    pI::IntValue puttext_B (puttext_in[6]);
    pI::BoolValue puttext_do_draw (puttext_in[7]);

    puttext_in[0] = drawRectDepth_in[0];
    puttext_R.SetData (255);
    puttext_G.SetData (255);
    puttext_B.SetData (255);
    puttext_do_draw.SetData (pI_TRUE);

    puttext_x.SetData (285);
    puttext_y.SetData (272);

    // Executes getImageAndDepth and setTiltAndLED a first time to initialize Kinect.

    getImageAndDepth->Execute (getImageAndDepth_in, getImageAndDepth_out);

    pI::IntValue newTilt (setTiltAndLED_in[0]);
    pI::IntValue newLED (setTiltAndLED_in[1]);

    newTilt.SetData (0); // Tilt angle 0 degree
    newLED.SetData (1); // LED status Green

    setTiltAndLED->Execute (setTiltAndLED_in, setTiltAndLED_out);

    Poco::Timestamp now;

    bool doLoop = true;
    bool showDistance = false;

    try {
        while (doLoop) {
            while (!kbhit()) {

                // Get Depth and RGB data, ...
                getImageAndDepth->Execute (getImageAndDepth_in, getImageAndDepth_out);

                // ... apply depth to color image conversion, ...
                applyRGB->Execute (applyRGB_in, applyRGB_out);

                // ... draw a white rectangle into the center of both image and map, ...
                if (showDistance) {
                    drawRectImage->Execute (drawRectImage_in, drawRectImage_out);
                    drawRectDepth->Execute (drawRectDepth_in, drawRectDepth_out);
                }

                // ... and perhaps the result of distance measurement, ...
                double avgDistance = 0.;

                if (showDistance) {
                    pI::ShortMatrix depth (getImageAndDepth_out[1]);
                    double sumDistance = 0.;
                    int sumCount = 0;

                    for (int j = 230; j <= 250; ++j) {
                        for (int i = 310; i <= 330; ++i) {
                            int dep = depth.GetData (j, i);

                            if (dep < 2047) {
                                sumDistance += RawDepthToMeters (dep);
                                ++sumCount;
                            }
                        }
                    }

                    if (sumCount > 0) {
                        avgDistance = sumDistance / (double) sumCount;
                    }

                    char str[32];
                    sprintf (str, "%.2f m", avgDistance);
                    puttext_text.SetData (str);

                    puttext->Execute (puttext_in, puttext_out);
                }

                // ... and display both the image and the depth map.
                displayImage->Execute (displayImage_in, displayImage_out);
                displayDepth->Execute (displayDepth_in, displayDepth_out);

                std::cout << "Tilt " << (int) newTilt.GetData();
                std::cout << "; " << (1000000. / now.elapsed()) << " frames/sec          \r";

                now.update();
            }

            int ch = getch();

            switch (ch) {
                case 'q':
                case 'Q':
                case 27 /* ESC */:
                    // Terminates
                    newTilt.SetData (0); // Tilt angle 0 degree
                    newLED.SetData (0); // LED status off

                    setTiltAndLED->Execute (setTiltAndLED_in, setTiltAndLED_out);

                    doLoop = false;
                    break;

                case 'm':
                case 'M': // Measurement mode
                    showDistance = !showDistance;
                    break;

                case 's':
                case 'S': { // Takes snapshot

                    std::string dateTime = Poco::DateTimeFormatter::format (Poco::LocalDateTime(), "%Y-%m-%d_%H-%M-%S");

                    std::string name = std::string ("KinectImage_") + dateTime + std::string (".png");
                    fileName.SetData (const_cast<char*> (name.c_str()));

                    saveImage->Execute (saveImage_in);

                    std::cout << "Saved " << name << std::endl;

                    // HACK Experimental: saves PCL point cloud

                    pI::ShortMatrix depth (getImageAndDepth_out[1]);
                    int width = depth.GetCols();
                    int height = depth.GetRows();

                    pcl::PointCloud<pcl::PointXYZ> cloud;

                    for (int j = 0; j < height; ++j) {
                        for (int i = 0; i < width; ++i) {
                            int dep = depth.GetData (j, i);

                            if (dep < 2048) {
                                cloud.push_back (DepthToWorld (i, j, dep));
                            }
                        }
                    }

                    name = std::string ("PointCloud_") + dateTime + std::string (".pcd");
                    pcl::io::savePCDFileASCII (name, cloud);

                    std::cout << "Saved " << name << std::endl;
                }
                break;
                case '+':
                case 'u':
                case 'U':
                    //std::cout << std::endl << "Tilt: " << newTilt.GetData();
                    newTilt.SetData (newTilt.GetData() + 1);
                    //std::cout << " -> " << newTilt.GetData() << std::endl;
                    setTiltAndLED->Execute (setTiltAndLED_in, setTiltAndLED_out);
                    break;
                case '-':
                case 'd':
                case 'D':
                    //std::cout << std::endl << "Tilt: " << newTilt.GetData();
                    newTilt.SetData (newTilt.GetData() - 1);
                    //std::cout << " -> " << newTilt.GetData() << std::endl;
                    setTiltAndLED->Execute (setTiltAndLED_in, setTiltAndLED_out);
                    break;
                default:
                    break;
            }
        }
    } catch (pI::exception::ExecutionException& e) {
        std::cerr << e.what() << std::endl;
    }
}
