#include <pcpred/import/bvh_importer.h>

#include <stdio.h>
#include <string.h>

using namespace pcpred;


bool BvhImporter::import(const char* filename)
{
    FILE* fp = fopen(filename, "r");
    if (fp == 0)
    {
        return false;
    }

    initialize();
    readHierarchy(fp);
    readMotion(fp);

    fclose(fp);

    return true;
}

void BvhImporter::initialize()
{
    frame_time_ = 0.;
    num_frames_ = 0;

    joint_names_.clear();
    joint_name_to_index_map_.clear();

    parents_.clear();
    children_.clear();

    joint_channels_.clear();

    transformations_.clear();
}

void BvhImporter::readHierarchy(FILE*& fp)
{
    // dummy string "HIERARCHY"
    fscanf(fp, "%*s");

    // read from root
    readHierarchyJoint(fp, -1);
}

void BvhImporter::readHierarchyJoint(FILE*& fp, int parent_index)
{
    char buffer[1024];

    const int index = joint_names_.size();

    // JOINT (name) {
    fscanf(fp, "%*s%s%*s", buffer);
    joint_names_.push_back(buffer);
    joint_name_to_index_map_[buffer] = index;
    parents_.push_back(parent_index);

    while (true)
    {
        fscanf(fp, "%s", buffer);

        if (strcmp(buffer, "OFFSET") == 0)
        {
            double x, y, z;
            fscanf(fp, "%lf%lf%lf", &x, &y, &z);
            offsets_.push_back(Eigen::Vector3d(x, y, z));
        }

        else if (strcmp(buffer, "CHANNELS"))
        {
            int num_channels;
            fscanf(fp, "%d", &num_channels);

            std::vector<JointChannelType> channels;
            for (int i=0; i<num_channels; i++)
            {
                fscanf(fp, "%s", buffer);

                if (strcmp(buffer, "Xposition")) channels.push_back(JointChannelXposition);
                else if (strcmp(buffer, "Yposition")) channels.push_back(JointChannelYposition);
                else if (strcmp(buffer, "Zposition")) channels.push_back(JointChannelZposition);
                else if (strcmp(buffer, "Xrotation")) channels.push_back(JointChannelXrotation);
                else if (strcmp(buffer, "Yrotation")) channels.push_back(JointChannelYrotation);
                else if (strcmp(buffer, "Zrotation")) channels.push_back(JointChannelZrotation);
            }
            joint_channels_.push_back(channels);
        }

        else if (strcmp(buffer, "JOINTS"))
        {
            fseek(fp, -6, SEEK_CUR);
            readHierarchyJoint(fp, index);
        }

        else if (strcmp(buffer, "End"))
        {
            // currently ignoring end sites
            fscanf(fp, "%*s%*s%*s%*s%*s%*s%*s");
        }

        else if (strcmp(buffer, "}"))
        {
            break;
        }
    }
}

void BvhImporter::readMotion(FILE*& fp)
{
    // MOTION
    fscanf(fp, "%*s");

    // Frames:
    fscanf(fp, "%*s%d", &num_frames_);

    // Frame Time:
    fscanf(fp, "%*s%*s%lf", &frame_time_);

    for (int i=0; i<num_frames_; i++)
    {
        transformations_.push_back(std::vector<Eigen::Affine3d>(0));

        for (int j=0; j<joint_names_.size(); j++)
        {
            Eigen::Affine3d m = j == 0 ? Eigen::Affine3d::Identity() : transformations_[i][ parents_[j] ];

            for (int k=0; k<joint_channels_[j].size(); k++)
            {
                double value;
                fscanf(fp, "%lf", &value);

                switch(joint_channels_[j][k])
                {
                case JointChannelXposition:
                    m.translate(Eigen::Vector3d(value, 0.0, 0.0));
                    break;
                case JointChannelYposition:
                    m.translate(Eigen::Vector3d(0.0, value, 0.0));
                    break;
                case JointChannelZposition:
                    m.translate(Eigen::Vector3d(0.0, 0.0, value));
                    break;
                case JointChannelXrotation:
                    m.rotate(Eigen::AngleAxisd(value, Eigen::Vector3d(1.0, 0.0, 0.0)));
                    break;
                case JointChannelYrotation:
                    m.rotate(Eigen::AngleAxisd(value, Eigen::Vector3d(0.0, 1.0, 0.0)));
                    break;
                case JointChannelZrotation:
                    m.rotate(Eigen::AngleAxisd(value, Eigen::Vector3d(0.0, 0.0, 1.0)));
                    break;
                }
            }

            transformations_[i].push_back(m);
        }
    }
}

Eigen::Affine3d BvhImporter::jointTransformation(int frame_index, int joint_index)
{
    return transformations_[frame_index][joint_index];
}

Eigen::Affine3d BvhImporter::jointTransformation(int frame_index, const std::string& joint_name)
{
    return transformations_[frame_index][ joint_name_to_index_map_[joint_name] ];
}

std::vector<Eigen::Vector3d> BvhImporter::childrenOffsets(int joint_index)
{
    std::vector<Eigen::Vector3d> offsets;
    for (int i=0; i<children_[joint_index].size(); i++)
        offsets.push_back(offsets_[children_[joint_index][i]]);

    return offsets;
}
