/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
*/

#ifndef _GAZEBO_RENDERING_GPUSONAR_PRIVATE_HH_
#define _GAZEBO_RENDERING_GPUSONAR_PRIVATE_HH_

#include <string>
#include <vector>

#include "gazebo/rendering/RenderTypes.hh"
#include "GpuSonarRenderTypes.hh"

#include "gazebo/common/Event.hh"

namespace Ogre
{
  class Camera;
  class Material;
  class MovableObject;
  class RenderTarget;
  class SceneNode;
  class Texture;
  class Viewport;
}

namespace gazebo
{
  namespace common
  {
    class Mesh;
  }

  namespace rendering
  {
    /// \internal
    /// \brief Private data for the GpuSonar class
    class GpuSonarPrivate
    {
      /// \brief Event triggered when new sonar range data are available.
      /// \param[in] _frame New frame containing raw sonar data.
      /// \param[in] _width Width of frame.
      /// \param[in] _height Height of frame.
      /// \param[in] _depth Depth of frame.
      /// \param[in] _format Format of frame.
      public: event::EventT<void(const float *_frame, unsigned int _width,
                   unsigned int _height, unsigned int _depth,
                   const std::string &_format)> newSonarFrame;

      /// \brief Raw buffer of sonar data.
      public: float *sonarBuffer;

      /// \brief Outgoing sonar data, used by newSonarFrame event.
      public: float *sonarScan;

      /// \brief Pointer to Ogre material for the first rendering pass.
      public: Ogre::Material *matFirstPass;

      /// \brief Pointer to Ogre material for the sencod rendering pass.
      public: Ogre::Material *matSecondPass;

      /// \brief An array of first pass textures.
      public: Ogre::Texture *firstPassTextures[3];

      /// \brief Second pass texture.
      public: Ogre::Texture *secondPassTexture;

      /// \brief First pass render targets.
      public: Ogre::RenderTarget *firstPassTargets[3];

      /// \brief Second pass render target.
      public: Ogre::RenderTarget *secondPassTarget;

      /// \brief First pass viewports.
      public: Ogre::Viewport *firstPassViewports[3];

      /// \brief Second pass viewport
      public: Ogre::Viewport *secondPassViewport;

      /// \brief Number of first pass textures.
      public: unsigned int textureCount;

      /// \brief A list of camera angles for first pass rendering.
      public: double cameraYaws[4];

      /// \brief Temporary pointer to the current render target.
      public: Ogre::RenderTarget *currentTarget;

      /// \brief Temporary pointer to the current material.
      public: Ogre::Material *currentMat;

      /// \brief Ogre orthorgraphic camera used in the second pass for
      /// undistortion.
      public: Ogre::Camera *orthoCam;

      /// \brief Ogre scenenode where the orthorgraphic camera is attached to.
      public: Ogre::SceneNode *pitchNodeOrtho;

      /// \brief Ogre mesh used to create a canvas for undistorting range values
      /// in the second rendering pass.
      public: common::Mesh *undistMesh;

      /// \brief Ogre movable object created from the canvas mesh.
      public: Ogre::MovableObject *object;

      /// \brief Pointer to visual that holds the canvas.
      public: VisualPtr visual;

      /// \brief Image width of second pass.
      public: unsigned int w2nd;

      /// \brief Image height of second pass.
      public: unsigned int h2nd;

      /// \brief Time taken to complete the two rendering passes.
      public: double lastRenderDuration;

      /// \brief List of texture unit indices used during the second
      /// rendering pass.
      public: std::vector<int> texIdx;

      /// Number of second pass texture units created.
      public: static int texCount;
    };
  }
}
#endif
