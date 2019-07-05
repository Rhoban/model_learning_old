#include <tclap/CmdLine.h>

#include <hl_monitoring/replay_image_provider.h>
#include <hl_communication/utils.h>
#include <hl_labelling/video_labelling_manager.h>
#include <hl_labelling/labelling_manager.h>

#include <rhoban_model_learning/log_models/log_machine_model.h>
#include <rhoban_model_learning/camera_calibration/camera_model.h>

#include <rhoban_utils/util.h>
#include <rhoban_utils/history/history.h>
#include <rhoban_utils/tables/double_table.h>

using namespace TCLAP;

int main(int argc, char** argv)
{
  try
  {
    CmdLine cmd("Extract aruco markers from images and writes their position in a csv file ", ' ', "0.1");

    ValueArg<std::string> labelsArg("l", "labels_path", "path to label pb file", true, "",
                                    "Path to labels protobuf file.", cmd);

    ValueArg<std::string> cameraFromWorldArg("w", "world", "camera from world", true, "camera_from_world.pb",
                                             "camera from world", cmd);

    cmd.parse(argc, argv);

    rhoban_utils::HistoryCollection histories;
    histories.pose("camera_from_world");
    histories.boolean("is_valid");
    histories.number("px_std_dev");
    std::vector<double> sequences_timestamps;
    std::vector<std::vector<double>> labels;

    std::string camera_from_world_path = cameraFromWorldArg.getValue();
    hl_monitoring::ReplayImageProvider replay_image_provider;
    replay_image_provider.loadMetaInformation(camera_from_world_path);
    hl_communication::VideoMetaInformation video_meta_information = replay_image_provider.getMetaInformation();

    // XXX needed ?
    if (!video_meta_information.has_source_id())
    {
      throw std::runtime_error(DEBUG_INFO + " no source id in meta_information");
    }

    // constructing :
    //   - camera_from_world history
    //   - is_valid history
    //   - the vector of the timestamps at which the log sequences starts
    //   - correspondance between frame index and timestamps (used later)
    std::vector<double> timestamps;
    bool was_not_valid = true;
    for (const hl_communication::FrameEntry& frame_entry : video_meta_information.frames())
    {
      if (frame_entry.has_pose())
      {
        // camera_from_world history
        const hl_communication::Pose3D& pose = frame_entry.pose();
        Eigen::Affine3d camera_from_world = getAffineFromProtobuf(pose);
        histories.pose("camera_from_world")->pushValue(frame_entry.utc_ts(), camera_from_world);
        histories.number("px_std_dev")->pushValue(frame_entry.utc_ts(), 5);

        // is_valid history
        bool is_valid = frame_entry.status() == hl_communication::FrameStatus::MOVING;
        histories.boolean("is_valid")->pushValue(frame_entry.utc_ts(), is_valid);

        // vecor of timestamps at which log sequences starts
        if (is_valid)
        {
          if (was_not_valid)
            sequences_timestamps.push_back(frame_entry.utc_ts());
          was_not_valid = false;
        }
        else
          was_not_valid = true;
      }
      // timestamps vector (correspondance frame index timestamp)
      timestamps.push_back(frame_entry.utc_ts());
    }

    // constructing the map : timestamp -> ((x,y,z,px,py), (x,y,z,px,py), ...) x,y,z in field
    std::string labels_path = labelsArg.getValue();
    hl_communication::MovieLabelCollection movie_label_collection;
    hl_communication::readFromFile(labels_path, &movie_label_collection);
    double ball_radius = 10;
    hl_labelling::LabellingManager labelling_manager(ball_radius);
    labelling_manager.importLabels(movie_label_collection);

    // hl_communication::MovieLabelCollection movie_labels;
    // hl_communication::readFromFile(path, &labels);

    for (hl_communication::LabelCollection label_collection : movie_label_collection.label_collections())
    {
      for (hl_communication::LabelMsg label_msg : label_collection.labels())
      {
        double timestamp = timestamps[label_msg.frame_index()];
        for (hl_communication::Match2D3DMsg match_2d_3d_msg : label_msg.field_matches())
        {
          cv::Point2f img_point = protobufToCV(match_2d_3d_msg.img_pos());
          cv::Point3f obj_point_in_field = protobufToCV(match_2d_3d_msg.obj_pos());
          std::vector<double> label;
          label.push_back(timestamp);
          label.push_back(obj_point_in_field.x);
          label.push_back(obj_point_in_field.y);
          label.push_back(obj_point_in_field.z);
          label.push_back(img_point.x);
          label.push_back(img_point.y);
          labels.push_back(label);
        }
      }
    }

    Eigen::MatrixXd data(labels.size(), 6);
    for (size_t i = 0; i < labels.size(); i++)
    {
      data(i, 0) = labels[i][0];
      data(i, 1) = labels[i][1];
      data(i, 2) = labels[i][2];
      data(i, 3) = labels[i][3];
      data(i, 4) = labels[i][4];
      data(i, 5) = labels[i][5];
    }

    rhoban_utils::DoubleTable double_table({ "timestamp", "img_x ", " img_y ", " pos_x ", " pos_y ", " pos_z " }, data);
    double_table.writeFile("labels_data.csv");
    // creating the initial log_machine_model
    rhoban_model_learning::LogMachineModel log_machine_model(histories, sequences_timestamps);
    log_machine_model.saveFile("log_machine_model.json", true);
  }
  catch (const ArgException& exc)
  {
    std::cerr << "error: " << exc.what() << " for arg " << exc.argId() << std::endl;
    exit(EXIT_FAILURE);
  }
}
