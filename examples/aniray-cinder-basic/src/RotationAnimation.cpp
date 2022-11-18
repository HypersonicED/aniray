/* RotationAnimation.cpp: Animation for example program aniray-cinder-basic
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

#include <cmath>
#include <memory>

#include "cinder/gl/gl.h"
#include "cinder/Vector.h"

#include <aniray/Geometry.hpp>
#include <aniray/NodeAnimation.hpp>

using namespace aniray;

template <typename NodeArrayT, typename NodeColor>
class RotationAnimation : aniray::NodeAnimation<NodeArrayT> {
	using NodeAnimation = aniray::NodeAnimation<NodeArrayT>;
	using Point = aniray::Point;
	using NodeAnimation::mNodeArray;
	using NodeT = typename NodeArrayT::InnerNodeT;
    using NodeColorT = typename NodeArrayT::InnerNodeT::InnerDataT;

	public:
		RotationAnimation(NodeArrayT &nodeArray, Point origin,
						  double radius, double periodMilliseconds, double height, double decay)
			: NodeAnimation::NodeAnimation(nodeArray)
			, mOrigin(origin)
			, mRadius(radius)
			, mPeriodMilliseconds(periodMilliseconds)
			, mHeight(height)
			, mDecay(decay) {}

		void frame(double milliseconds) override {
			NodeAnimation::frame(milliseconds);

			static double lastMilliseconds = 0;
			double angle = (std::fmod(milliseconds, mPeriodMilliseconds) / mPeriodMilliseconds) * (2 * boost::math::constants::pi<double>());
			mCurrentPosition = Point(
				mRadius * std::cos(angle),
				mHeight,
				mRadius * std::sin(angle));
			boost::geometry::add_point(mCurrentPosition, mOrigin);

			float decay = static_cast<float>((milliseconds - lastMilliseconds) / mDecay);
			for(std::shared_ptr<NodeT> node: mNodeArray.nodes()) {
				NodeColor outColor = node->data() - NodeColor({decay, -decay, 0});
				if (outColor[0] < 0) {
					outColor = NodeColor({0, 1, 0});
				}
				node->data(outColor);
			}
			mNodeArray.findNodesInRadiusOfSource(
				mCurrentPosition,
				[this](const std::shared_ptr<NodeT> targetNode) -> bool {
					targetNode->data(NodeColorT({255,0,0}));
					return false;
				},
				true, mRadius);
			lastMilliseconds = milliseconds;
		}
		void draw() {
			cinder::gl::color(cinder::ColorA(cinder::CM_RGB, 1, 0, 0, 0.1));
			cinder::gl::drawSphere(glm::vec3(
				mCurrentPosition.x(),
				mCurrentPosition.y(),
				mCurrentPosition.z()
			), mRadius);
		}
	private:
		double mRadius;
		double mPeriodMilliseconds;
		double mHeight;
		double mDecay;
		Point mOrigin;
		Point mCurrentPosition;
};
