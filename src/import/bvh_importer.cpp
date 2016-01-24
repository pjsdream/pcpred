#include <pcpred/import/bvh_importer.h>
#include <resource_retriever/retriever.h>

#include <ros/ros.h>

#include <stdio.h>
#include <string.h>

using namespace pcpred;


const double BvhImporter::DEGREES_TO_RADIANS = M_PI / 180.;


bool BvhImporter::import(const char* filename)
{
    FILE* fp = fopen(filename, "r");
    if (fp == NULL)
    {
        char package_filename[128];
        sprintf(package_filename, "package://pcpred/data/bvh/%s", filename);

        resource_retriever::Retriever retriever;
        resource_retriever::MemoryResource resource;

        try
        {
            resource = retriever.get(package_filename);
        }
        catch (resource_retriever::Exception& e)
        {
            ROS_ERROR("Failed to retrieve file: %s", filename);
            ROS_ERROR("Failed to retrieve file: %s", e.what());
            return false;
        }

        std::istringstream s((const char *)resource.data.get());

        initialize();
        readHierarchy(s);
        readMotion(s);

        return true;
    }

    else
    {
        initialize();
        readHierarchy(fp);
        readMotion(fp);

        fclose(fp);

        return true;
    }
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

void BvhImporter::readHierarchy(std::istringstream& s)
{
    char buffer[128];

    // dummy string "HIERARCHY"
    s >> buffer;

    // read from root
    readHierarchyJoint(s, -1);
}

void BvhImporter::readHierarchyJoint(FILE*& fp, int parent_index)
{
    char buffer[1024];


    // JOINT (name) {
    fscanf(fp, "%*s%s%*s", buffer);

    const int index = joint_names_.size();
    const std::string joint_name = buffer;

    joint_names_.push_back(joint_name);
    joint_name_to_index_map_[joint_name] = index;
    parents_.push_back(parent_index);
    children_.push_back(std::vector<int>(0));

    if (parent_index != -1)
        children_[parent_index].push_back(index);

    while (true)
    {
        fscanf(fp, "%s", buffer);

        if (strcmp(buffer, "OFFSET") == 0)
        {
            double x, y, z;
            fscanf(fp, "%lf%lf%lf", &x, &y, &z);
            offsets_.push_back(Eigen::Vector3d(x, y, z));
        }

        else if (strcmp(buffer, "CHANNELS") == 0)
        {
            int num_channels;
            fscanf(fp, "%d", &num_channels);

            std::vector<JointChannelType> channels;
            for (int i=0; i<num_channels; i++)
            {
                fscanf(fp, "%s", buffer);

                if (strcmp(buffer, "Xposition") == 0) channels.push_back(JointChannelXposition);
                else if (strcmp(buffer, "Yposition") == 0) channels.push_back(JointChannelYposition);
                else if (strcmp(buffer, "Zposition") == 0) channels.push_back(JointChannelZposition);
                else if (strcmp(buffer, "Xrotation") == 0) channels.push_back(JointChannelXrotation);
                else if (strcmp(buffer, "Yrotation") == 0) channels.push_back(JointChannelYrotation);
                else if (strcmp(buffer, "Zrotation") == 0) channels.push_back(JointChannelZrotation);
            }
            joint_channels_.push_back(channels);
        }

        else if (strcmp(buffer, "JOINT") == 0)
        {
            fseek(fp, -5, SEEK_CUR);
            readHierarchyJoint(fp, index);
        }

        else if (strcmp(buffer, "End") == 0)
        {
            // Site { OFFSET %lf %lf %lf }
            double x, y, z;
            fscanf(fp, "%*s%*s%*s%lf%lf%lf%*s", &x, &y, &z);

            const std::string endsite_name = joint_name + "EndSite";
            const int endsite_index = joint_names_.size();

            joint_names_.push_back(endsite_name);
            joint_name_to_index_map_[endsite_name] = endsite_index;
            parents_.push_back(index);
            children_.push_back(std::vector<int>(0));

            children_[index].push_back(endsite_index);

            offsets_.push_back(Eigen::Vector3d(x, y, z));
            joint_channels_.push_back(std::vector<JointChannelType>(0));
        }

        else if (strcmp(buffer, "}") == 0)
        {
            break;
        }
    }
}

void BvhImporter::readHierarchyJoint(std::istringstream& s, int parent_index)
{
    char buffer[1024];
    char dummy[128];


    // JOINT (name) {
    s >> dummy >> buffer >> dummy;

    const int index = joint_names_.size();
    const std::string joint_name = buffer;

    joint_names_.push_back(joint_name);
    joint_name_to_index_map_[joint_name] = index;
    parents_.push_back(parent_index);
    children_.push_back(std::vector<int>(0));

    if (parent_index != -1)
        children_[parent_index].push_back(index);

    while (true)
    {
        s >> buffer;

        if (strcmp(buffer, "OFFSET") == 0)
        {
            double x, y, z;
            s >> x >> y >> z;
            offsets_.push_back(Eigen::Vector3d(x, y, z));
        }

        else if (strcmp(buffer, "CHANNELS") == 0)
        {
            int num_channels;
            s >> num_channels;

            std::vector<JointChannelType> channels;
            for (int i=0; i<num_channels; i++)
            {
                s >> buffer;

                if (strcmp(buffer, "Xposition") == 0) channels.push_back(JointChannelXposition);
                else if (strcmp(buffer, "Yposition") == 0) channels.push_back(JointChannelYposition);
                else if (strcmp(buffer, "Zposition") == 0) channels.push_back(JointChannelZposition);
                else if (strcmp(buffer, "Xrotation") == 0) channels.push_back(JointChannelXrotation);
                else if (strcmp(buffer, "Yrotation") == 0) channels.push_back(JointChannelYrotation);
                else if (strcmp(buffer, "Zrotation") == 0) channels.push_back(JointChannelZrotation);
            }
            joint_channels_.push_back(channels);
        }

        else if (strcmp(buffer, "JOINT") == 0)
        {
            s.seekg(-5, std::ios_base::cur);
            readHierarchyJoint(s, index);
        }

        else if (strcmp(buffer, "End") == 0)
        {
            // Site { OFFSET %lf %lf %lf }
            double x, y, z;
            s >> dummy >> dummy >> dummy >> x >> y >> z >> dummy;

            const std::string endsite_name = joint_name + "EndSite";
            const int endsite_index = joint_names_.size();

            joint_names_.push_back(endsite_name);
            joint_name_to_index_map_[endsite_name] = endsite_index;
            parents_.push_back(index);
            children_.push_back(std::vector<int>(0));

            children_[index].push_back(endsite_index);

            offsets_.push_back(Eigen::Vector3d(x, y, z));
            joint_channels_.push_back(std::vector<JointChannelType>(0));
        }

        else if (strcmp(buffer, "}") == 0)
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
            m.translate(offsets_[j]);

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
                    m.rotate(Eigen::AngleAxisd(value * DEGREES_TO_RADIANS, Eigen::Vector3d(1.0, 0.0, 0.0)));
                    break;
                case JointChannelYrotation:
                    m.rotate(Eigen::AngleAxisd(value * DEGREES_TO_RADIANS, Eigen::Vector3d(0.0, 1.0, 0.0)));
                    break;
                case JointChannelZrotation:
                    m.rotate(Eigen::AngleAxisd(value * DEGREES_TO_RADIANS, Eigen::Vector3d(0.0, 0.0, 1.0)));
                    break;
                }
            }

            transformations_[i].push_back(m);
        }
    }
}

void BvhImporter::readMotion(std::istringstream& s)
{
    char dummy[128];

    // MOTION
    s >> dummy;

    // Frames:
    s >> dummy >> num_frames_;

    // Frame Time:
    s >> dummy >> dummy >> frame_time_;

    for (int i=0; i<num_frames_; i++)
    {
        transformations_.push_back(std::vector<Eigen::Affine3d>(0));

        for (int j=0; j<joint_names_.size(); j++)
        {
            Eigen::Affine3d m = j == 0 ? Eigen::Affine3d::Identity() : transformations_[i][ parents_[j] ];
            m.translate(offsets_[j]);

            for (int k=0; k<joint_channels_[j].size(); k++)
            {
                double value;
                s >> value;

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
                    m.rotate(Eigen::AngleAxisd(value * DEGREES_TO_RADIANS, Eigen::Vector3d(1.0, 0.0, 0.0)));
                    break;
                case JointChannelYrotation:
                    m.rotate(Eigen::AngleAxisd(value * DEGREES_TO_RADIANS, Eigen::Vector3d(0.0, 1.0, 0.0)));
                    break;
                case JointChannelZrotation:
                    m.rotate(Eigen::AngleAxisd(value * DEGREES_TO_RADIANS, Eigen::Vector3d(0.0, 0.0, 1.0)));
                    break;
                }
            }

            transformations_[i].push_back(m);
        }
    }
}

void BvhImporter::scale(double s)
{
    for (int i=0; i<joint_names_.size(); i++)
        offsets_[i] *= s;

    for (int i=0; i<num_frames_; i++)
    {
        for (int j=0; j<joint_names_.size(); j++)
            transformations_[i][j].translation() *= s;
    }
}

void BvhImporter::rotate(double angle, const Eigen::Vector3d& axis)
{
    Eigen::AngleAxisd rotation(angle, axis);

    for (int i=0; i<num_frames_; i++)
    {
        for (int j=0; j<joint_names_.size(); j++)
            transformations_[i][j].prerotate(rotation);
    }
}

void BvhImporter::translate(const Eigen::Vector3d& t)
{
    for (int i=0; i<num_frames_; i++)
    {
        for (int j=0; j<joint_names_.size(); j++)
            transformations_[i][j].pretranslate(t);
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

Eigen::Affine3d BvhImporter::jointTransformation(double time, int joint_index)
{
    const int frame_index0 = time / frame_time_;
    const int frame_index1 = frame_index0 + 1;
    const double t = time / frame_time_ - frame_index0;

    if (frame_index1 < num_frames_)
    {
        const Eigen::Affine3d m0 = transformations_[frame_index0][joint_index];
        const Eigen::Affine3d m1 = transformations_[frame_index1][joint_index];

        const Eigen::Quaterniond q0(m0.linear());
        const Eigen::Quaterniond q1(m1.linear());
        const Eigen::Quaterniond q = q0.slerp(t, q1);

        const Eigen::Vector3d p = (1.-t) * m0.translation() + t * m1.translation();

        Eigen::Affine3d m = Eigen::Affine3d::Identity();
        m.translate(p).rotate(q);
        return m;
    }

    else
        return transformations_[num_frames_ - 1][joint_index];
}

Eigen::Affine3d BvhImporter::jointTransformation(double time, const std::string& joint_name)
{
    return jointTransformation(time, joint_name_to_index_map_[joint_name]);
}

std::vector<Eigen::Vector3d> BvhImporter::childrenOffsets(int joint_index)
{
    std::vector<Eigen::Vector3d> offsets;
    for (int i=0; i<children_[joint_index].size(); i++)
        offsets.push_back(offsets_[children_[joint_index][i]]);

    return offsets;
}
