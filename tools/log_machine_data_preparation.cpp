#include <tclap/CmdLine.h>

#include <hl_monitoring/replay_image_provider.h>
#include <hl_communication/utils.h>
#include <hl_labelling/video_labelling_manager.h>

using namespace TCLAP;

int main(int argc, char** argv)
{
  try
  {
    CmdLine cmd("Extract aruco markers from images and writes their position in a csv file ", ' ', "0.1");

    ValueArg<std::string> labelsArg("l", "labels_path", "path to label pb file", true, "",
                                    "Path to labels protobuf file.", cmd);

    ValueArg<std::string> firstGuessArg("p", "prior", "camera from field", true, "camera_from_field.pb",
                                        "camera from field", cmd);

    cmd.parse(argc, argv);

    std::string labels_path = labelsArg.getValue();
    // std::string metadata_path = metadataArg.getValue();
    std::string first_guess_path = firstGuessArg.getValue();

    hl_monitoring::ReplayImageProvider first_guess;
    first_guess.loadMetaInformation(first_guess_path);

    hl_communication::MovieLabelCollection labels;
    hl_communication::readFromFile(labels_path, &labels);
    hl_labelling::VideoLabellingManager video_labelling_manager;
    video_labelling_manager.importProtobuf(labels);

    int max_frame_id = first_guess.getNbFrames();
    for (size_t i = 0; i < max_frame_id; i++)
    {
      hl_communication::CameraInformation info = first_guess.getCameraMetaInformation(i);
    }
  }
  catch (const ArgException& exc)
  {
    std::cerr << "error: " << exc.what() << " for arg " << exc.argId() << std::endl;
    exit(EXIT_FAILURE);
  }
}
