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
                for (size_t i = 0; i < models.size(); i++)
                {
                    const RaveModel& raveModel = models.at(i);
                    for (size_t j = 0; j < raveModel.models.size(); j++)
                    {
                        flatModels.push_back(raveModel.models.at(j));
                    }
                }
            }

            inline void CreateModels(const Shader& shader, const OpenRAVE::RobotBase::ManipulatorPtr& manip)
            {
                OpenRAVE::RobotBasePtr robot = manip->GetRobot();
                std::vector<int> indices = manip->GetArmIndices();

                std::vector<OpenRAVE::KinBody::LinkPtr> affectedLinks;

                for (size_t i = 0; i < robot->GetLinks().size(); i++)
                {
                    const OpenRAVE::KinBody::LinkPtr& link = robot->GetLinks().at(i);
                    for (size_t j = 0; j < indices.size(); j++)
                    {
                        int idx = indices.at(j);
                        if( robot->DoesAffect(robot->GetJointFromDOFIndex(idx)->GetJointIndex(), link->GetIndex()))
                        {
                            affectedLinks.push_back(link);
                        }
                    }
                }

                for (size_t i = 0; i < affectedLinks.size(); i++)
                {
                    OpenRAVE::KinBody::LinkPtr link = affectedLinks.at(i);
                    models.push_back(CreateModel(shader, robot, link, ColorIntToRaveColor(colorTable[link->GetIndex()])));
                }
            }

            inline void CreateModels(const Shader& shader, const OpenRAVE::KinBodyPtr& body, const OpenRAVE::Vector& color)
            {
                for (size_t i = 0; i < body->GetLinks().size(); i++)
                {
                    const OpenRAVE::KinBody::LinkPtr& link = body->GetLinks().at(i);
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

                for (size_t i = 0; i < link->GetGeometries().size(); i++)
                {
                    const OpenRAVE::KinBody::Link::GeometryConstPtr& geom = link->GetGeometries().at(i);
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

                model.buffer->position_data.resize(mesh.vertices.size() * 3, 9999);
                model.buffer->color_data.resize(mesh.vertices.size() * 3);
                model.buffer->index_data.resize(mesh.indices.size());

                for (size_t i = 0; i < mesh.vertices.size(); i++)
                {
                    const OpenRAVE::Vector& vertex = mesh.vertices.at(i);
                    model.buffer->position_data[i * 3 + 0] = vertex.x;
                    model.buffer->position_data[i * 3 + 1] = vertex.y;
                    model.buffer->position_data[i * 3 + 2] = vertex.z;
                    model.buffer->color_data[i * 3 + 0] = color.x;
                    model.buffer->color_data[i * 3 + 1] = color.y;
                    model.buffer->color_data[i * 3 + 2] = color.z;
                }

                for (size_t i = 0; i < mesh.indices.size(); i++)
                {
                    int idx = mesh.indices.at(i);
                    model.buffer->index_data[i] = static_cast<unsigned short>(idx);
                }
                model.buffer->Initialize(shader);
                return model;
            }

            void UpdateModels()
            {
                for (size_t m = 0; m < models.size(); m++)
                {
                    RaveModel& model = models.at(m);
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
