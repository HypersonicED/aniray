/* main.cpp: Example program aniray-cinder-basic
 *
 * Created by Perry Naseck on 2022-11-17.
 *
 * This file is a part of Aniray
 * https://github.com/HypersonicED/aniray
 *
 * Copyright (c) 2022, Hypersonic
 * Copyright (c) 2022, Perry Naseck
 * All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <numeric>
#include <string>
#include <vector>

#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/Batch.h"
#include "cinder/Camera.h"
#include "cinder/CameraUi.h"
#include "cinder/GeomIo.h"
#include "cinder/Log.h"

#include <aniray/Geometry.hpp>
#include <aniray/Node.hpp>
#include <aniray/NodeArray.hpp>
#include <aniray/NodeArrayOutputOLA.hpp>
#include <aniray/NodeArraySampler.hpp>
#include <aniray/NodeAnimation.hpp>

#include "RotationAnimation.cpp"

using namespace aniray;
using namespace ci;
using namespace ci::app;

const bool ENABLE_OUTPUT = true;
const float MAX_FRAME_RATE = 240;
const std::string LED_CSV_FILE_NAME = "leds.csv";
const double SAMPLE_GRID_SPACING = 0.5;
const std::int64_t OUTPUT_OLA_INTERVAL_US = 27500;
const int WINDOW_WIDTH = 1920;
const int WINDOW_HEIGHT = 1080;
const int SPHERE_SUBDIVISIONS = 30;
const int SPHERE_RADIUS = 5;
const float DRAW_PHYSICAL_NODE_SIZE = 0.125f/5;
const float DRAW_SAMPLE_NODE_SIZE = 0.125f/20;

class ExampleAnirayCinderBasic : public ci::app::App
{
    // This helper function is used by NodeArrayOutput (and its children) to
    // convert your data type of choice for each Node into an array of values
    // to be sent as output.
	static std::array<std::uint8_t, 3> colorToOutput(ci::Color color) {
		return std::array<std::uint8_t, 3>({
			static_cast<std::uint8_t>(color[0]),
			static_cast<std::uint8_t>(color[1]),
			static_cast<std::uint8_t>(color[2])});
	}

    // Helpers for types with template parameters already included
	using Node = aniray::Node<ci::Color>;
	using NodeArray = aniray::NodeArray<Node>;
	using NodeArrayOutputOLA = aniray::NodeArrayOutputOLA<NodeArray, colorToOutput>;
	using NodeArraySampler = aniray::NodeArraySampler<NodeArray>;

  public:
    // Cinder methods
	static void prepareSettings (ci::app::App::Settings* settings);
	void resetCamera();
	void setup() override;
	void keyDown(ci::app::KeyEvent event) override;
    void mouseDown(ci::app::MouseEvent event) override;
	void mouseDrag(ci::app::MouseEvent event) override;
	void mouseWheel(ci::app::MouseEvent event) override;
	void draw() override;

    // Camera members for 3D space
	ci::CameraPersp	mCam;
    ci::CameraUi mCamUi;
    ci::gl::BatchRef mSphere;
	glm::vec3 center;

    // Animation and visualization vars
    bool mDrawLEDs = true;
    bool mDrawSamples = false;
    std::chrono::time_point<std::chrono::high_resolution_clock> mStartTime;

    // Containers for Aniray objects
	std::unique_ptr<NodeArray> mNodeArray;
	std::unique_ptr<NodeArray> mNodeArraySamples;
	std::unique_ptr<NodeArrayOutputOLA> mNodeArrayOutput;
	std::unique_ptr<NodeArraySampler> mNodeArraySampler;
	std::unique_ptr<RotationAnimation<NodeArray>> mRotationAnimation;
};

void ExampleAnirayCinderBasic::prepareSettings(ci::app::App::Settings* settings)
{
    settings->setFrameRate(MAX_FRAME_RATE);
    settings->setResizable(false);
}

void ExampleAnirayCinderBasic::resetCamera() {
	mCam.lookAt(
		glm::vec3(center[0] + 0, center[1]/5 + 18, center[2] + 12),
		center
	);
}

void ExampleAnirayCinderBasic::setup()
{
    // Arrange the primary (physical) Nodes/LEDs using locations from a CSV file
	mNodeArray = std::make_unique<NodeArray>(getAssetPath(LED_CSV_FILE_NAME));

    // Find a denser grid of virtual Nodes/LEDs using the original physical grid
	mNodeArraySamples = std::make_unique<NodeArray>(*mNodeArray, true, SAMPLE_GRID_SPACING);

    // Output the physical Nodes via OLA
	if (ENABLE_OUTPUT) {
		mNodeArrayOutput = std::make_unique<NodeArrayOutputOLA>(*mNodeArray, OUTPUT_OLA_INTERVAL_US);
	}

    // Use the virtual Nodes to inform physical Nodes
	mNodeArraySampler = std::make_unique<NodeArraySampler>(*mNodeArray, *mNodeArraySamples);

    // A basic rotation animation
	mRotationAnimation = std::make_unique<RotationAnimation<NodeArray>>(*mNodeArraySamples, mNodeArray->center(), 2.5, 1.5, 4000, 0, 3000);

    // Cinder settings
    setWindowSize(WINDOW_WIDTH, WINDOW_HEIGHT);
	ci::gl::enableVerticalSync(false);
	auto lambert = gl::ShaderDef().lambert().color();
	gl::GlslProgRef shader = gl::getStockShader(lambert);
    auto sphere = geom::Sphere().subdivisions(SPHERE_SUBDIVISIONS).radius(SPHERE_RADIUS);
	mSphere = gl::Batch::create(sphere, shader);

    // Find the center and focus the camera on the center
	center = {
		mNodeArray->center().x(),
		mNodeArray->center().y(),
		mNodeArray->center().z()
	};
    mCamUi = CameraUi(&mCam);
	resetCamera();

    mStartTime = std::chrono::high_resolution_clock::now();
}

void ExampleAnirayCinderBasic::keyDown( KeyEvent event )
{
	if (event.getCode() == ci::app::KeyEvent::KEY_ESCAPE) quit();
	if (event.getChar() == 'r') resetCamera();
	if (event.getChar() == 'p') mDrawLEDs = !mDrawLEDs;
	if (event.getChar() == 's') mDrawSamples = !mDrawSamples;
}

void ExampleAnirayCinderBasic::mouseDown(ci::app::MouseEvent event)
{
	mCamUi.mouseDown(event);
}

void ExampleAnirayCinderBasic::mouseDrag(ci::app::MouseEvent event)
{
	mCamUi.mouseDrag(event);
}

void ExampleAnirayCinderBasic::mouseWheel(ci::app::MouseEvent event)
{
	mCamUi.mouseWheel(event);
}

void ExampleAnirayCinderBasic::draw()
{
    // New Cinder frame
	gl::clear();
	gl::enableDepthRead();
	gl::enableDepthWrite();
	gl::enableAlphaBlending(true);
	gl::setMatrices(mCam);

    // Run the next frame of the animation
    auto elapsedTime = std::chrono::high_resolution_clock::now() - mStartTime;
    auto elapsedTimeMs = std::chrono::duration_cast<std::chrono::milliseconds>(elapsedTime).count();
    mRotationAnimation->frame(elapsedTimeMs);
    mRotationAnimation->draw();

    // Sample the virtual Nodes, average the squares their RGB values, and assign to the physical Nodes
	mNodeArraySampler->sample([](std::shared_ptr<Node> const targetNode,
                                 std::vector<std::shared_ptr<Node>> const sources) -> ci::Color {
        std::vector<double> mixR;
        std::vector<double> mixG;
        std::vector<double> mixB;
        for (std::shared_ptr<Node> sourceNode : sources) {
            ci::Color sourceColor{};
            sourceColor = sourceNode->data();
            mixR.push_back(sourceColor[0] * sourceColor[0]);
            mixG.push_back(sourceColor[1] * sourceColor[1]);
            mixB.push_back(sourceColor[2] * sourceColor[2]);
        }
        ci::Color targetColor(
            {static_cast<float>(std::sqrt(std::accumulate(mixR.begin(), mixR.end(), 0.0) /
                        static_cast<double>(mixR.size()))),
            static_cast<float>(std::sqrt(std::accumulate(mixG.begin(), mixG.end(), 0.0) /
                        static_cast<double>(mixG.size()))),
            static_cast<float>(std::sqrt(std::accumulate(mixB.begin(), mixB.end(), 0.0) /
                        static_cast<double>(mixB.size())))});
        return targetColor;
    });

    // Draw physical Nodes
	if (mDrawLEDs) {
        for(std::shared_ptr<Node> node: mNodeArray->nodes()) {
			glm::vec3 offset(
				node->coords().x(),
				node->coords().y(),
				node->coords().z()
			);
			ci::Color nodeColor = node->data();

			gl::pushModelMatrix();
			gl::translate(offset);
			gl::scale(glm::vec3(DRAW_PHYSICAL_NODE_SIZE));
			if (node->ignore()) { gl::color(ColorA(CM_RGB, 0, 0, 0, 1)); }
			else { gl::color(ColorA(node->data(), 1)); }
			mSphere->draw();
			gl::popModelMatrix();
		}
	}

    // Draw virtual Nodes
	if (mDrawSamples) {
		for(std::shared_ptr<Node> node: mNodeArraySamples->nodes()) {
			glm::vec3 offset(
				node->coords().x(),
				node->coords().y(),
				node->coords().z()
			);
			ci::Color nodeColor = node->data();

			gl::pushModelMatrix();
			gl::translate(offset);
			gl::scale(glm::vec3(DRAW_SAMPLE_NODE_SIZE));
			gl::color(ColorA(node->data(), 1));
			mSphere->draw();
			gl::popModelMatrix();
		}
	}

    // Send to OLA output if enabled
	if (ENABLE_OUTPUT && !mNodeArrayOutput->updateAndSend()) {
		CI_LOG_E("Send DMX failed!");
	}

    // Cinder end of frame
	gl::disableDepthWrite();
	gl::disableDepthRead();
	gl::setMatricesWindow(ci::app::getWindowSize());
}

CINDER_APP(ExampleAnirayCinderBasic, ci::app::RendererGl, ExampleAnirayCinderBasic::prepareSettings);
