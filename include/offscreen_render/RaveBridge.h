#ifndef RAVEBRIDGE_H_
#define RAVEBRIDGE_H_

#include "OffscreenRenderer.h"
#include "Conversions.h"
#include <openrave/openrave.h>
#include <openrave/kinbody.h>

namespace offscreen_render
{
    class RaveBridge
    {
        public:
            static uint32_t colorTable[128];
            struct RaveModel
            {
                OpenRAVE::KinBodyPtr body;
                OpenRAVE::KinBody::LinkPtr link;
                std::vector<Model> models;
            };

            int GetColorIndex(uint8_t r, uint8_t g, uint8_t b, uint8_t a)
            {
                uint32_t c = ((uint32_t)r << 24) | ((uint32_t)g << 16) | ((uint32_t)b << 8) | (uint32_t)a;
                return GetColorIndex(c);
            }

            int GetColorIndex(uint32_t color)
            {
                for(int i = 0; i < 128; i++)
                {
                    if (color == colorTable[i])
                    {
                        return i;
                    }
                }
                return -1;
            }

            OpenRAVE::Vector ColorIntToRaveColor(int color)
            {
                uint8_t red =   (color & 0xFF000000) >> 24;
                uint8_t green = (color & 0x00FF0000) >> 16;
                uint8_t blue =  (color & 0x0000FF00) >> 8;
                uint8_t alpha = (color & 0x000000FF);

                return OpenRAVE::Vector((float)red / 256.0f, (float)green / 256.0f, (float)blue / 256.0f);
            }


            inline void GetAllModels(std::vector<Model>& flatModels)
            {
                for (const RaveModel& raveModel : models)
                {
                    for (const Model& model : raveModel.models)
                    {
                        flatModels.push_back(model);
                    }
                }
            }

            inline void CreateModels(const Shader& shader, const OpenRAVE::RobotBase::ManipulatorPtr& manip)
            {
                OpenRAVE::RobotBasePtr robot = manip->GetRobot();
                std::vector<int> indices = manip->GetArmIndices();

                std::vector<OpenRAVE::KinBody::LinkPtr> affectedLinks;

                for (const OpenRAVE::KinBody::LinkPtr& link : robot->GetLinks())
                {
                    for (int idx : indices)
                    {
                          if( robot->DoesAffect(robot->GetJointFromDOFIndex(idx)->GetJointIndex(), link->GetIndex()))
                          {
                              affectedLinks.push_back(link);
                          }
                    }
                }

                for (OpenRAVE::KinBody::LinkPtr link : affectedLinks)
                {
                    models.push_back(CreateModel(shader, robot, link, ColorIntToRaveColor(colorTable[link->GetIndex()])));
                }
            }

            inline void CreateModels(const Shader& shader, const OpenRAVE::KinBodyPtr& body, const OpenRAVE::Vector& color)
            {
                for(const OpenRAVE::KinBody::LinkPtr& link : body->GetLinks())
                {
                    models.push_back(CreateModel(shader, body, link, color));
                }

                bodies.push_back(body);
            }

            inline RaveModel CreateModel(const Shader& shader, const OpenRAVE::KinBodyPtr& body,
                    const OpenRAVE::KinBody::LinkPtr& link, const OpenRAVE::Vector& color)
            {
                RaveModel model;
                model.link = link;
                model.body = body;

                for (const OpenRAVE::KinBody::Link::GeometryConstPtr& geom : link->GetGeometries())
                {
                    model.models.push_back(CreateModel(shader, geom,  link->GetTransform() * geom->GetTransform(), color));
                }
                return model;
            }

            inline Model CreateModel(const Shader& shader, const OpenRAVE::KinBody::Link::GeometryConstPtr& geometry,
                    const OpenRAVE::Transform& globalTransform,
                    const OpenRAVE::Vector& color)
            {
                Model model;
                model.transform = ORToTransform(globalTransform).matrix();
                model.buffer = new VertexBuffer();

                const OpenRAVE::TriMesh& mesh = geometry->GetCollisionMesh();

                model.buffer->position_data.resize(mesh.vertices.size() * 3);
                model.buffer->color_data.resize(mesh.vertices.size() * 3);
                model.buffer->index_data.resize(mesh.indices.size());

                size_t i = 0;
                for (const OpenRAVE::Vector& vertex : mesh.vertices)
                {
                    model.buffer->position_data[i * 3 + 0] = vertex.x;
                    model.buffer->position_data[i * 3 + 1] = vertex.y;
                    model.buffer->position_data[i * 3 + 2] = vertex.z;
                    model.buffer->color_data[i * 3 + 0] = color.x;
                    model.buffer->color_data[i * 3 + 1] = color.y;
                    model.buffer->color_data[i * 3 + 2] = color.z;
                    i++;
                }

                i = 0;
                for (int idx : mesh.indices)
                {
                    model.buffer->index_data[i] = static_cast<unsigned short>(idx);
                    i++;
                }
                model.buffer->Initialize(shader);
                return model;
            }

            void UpdateModels()
            {
                for (RaveModel& model : models)
                {
                    for (size_t i = 0; i < model.models.size(); i++)
                    {
                        const OpenRAVE::KinBody::Link::GeometryConstPtr& geom = model.link->GetGeometry((int)i);
                        model.models[i].transform = ORToTransform(model.link->GetTransform() * geom->GetTransform()).matrix();
                    }
                }
            }


            std::vector<RaveModel> models;
            std::vector<OpenRAVE::KinBodyPtr> bodies;



    };
}



#endif // RAVEBRIDGE_H_ 
