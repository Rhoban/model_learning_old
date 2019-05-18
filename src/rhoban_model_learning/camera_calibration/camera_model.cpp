#include <rhoban_model_learning/camera_calibration/camera_model.h>

namespace rhoban_model_learning
{
CameraModel::CameraModel() : model()
{
}

CameraModel::CameraModel(const rhoban::CameraModel& camera_model) : model(camera_model)
{
}

CameraModel::CameraModel(const CameraModel& other) : model(other.model)
{
}

int CameraModel::getParametersSize() const
{
  return 9;
}

Eigen::VectorXd CameraModel::getParameters() const
{
  Eigen::VectorXd params(getParametersSize());
  int idx = 0;
  params(idx++) = model.getFocalX();
  params(idx++) = model.getFocalY();
  params(idx++) = model.getCenterX();
  params(idx++) = model.getCenterY();
  params.segment(idx, 5) = model.getDistortionCoeffsAsEigen();
  return params;
}

void CameraModel::setParameters(const Eigen::VectorXd& new_params)
{
  int idx = 0;
  model.setFocal(new_params.segment(idx, 2));
  idx += 2;
  model.setCenter(new_params.segment(idx, 2));
  idx += 2;
  model.setDistortion(new_params.segment(idx, 5));
  idx += 5;
}

std::vector<std::string> CameraModel::getParametersNames() const
{
  return { "focalX", "focalY", "centerX", "centerY", "k1", "k2", "p1", "p2", "k3" };
}

void CameraModel::fromJson(const Json::Value& json_value, const std::string& dir_name)
{
  model.fromJson(json_value, dir_name);
}
Json::Value CameraModel::toJson() const
{
  return model.toJson();
}

std::string CameraModel::getClassName() const
{
  return "CameraModel";
}

std::unique_ptr<Model> CameraModel::clone() const
{
  return std::unique_ptr<Model>(new CameraModel(*this));
}

}  // namespace rhoban_model_learning
