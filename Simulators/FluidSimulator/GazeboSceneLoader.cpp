#include "GazeboSceneLoader.h"
#include "Utilities/Logger.h"
#include <iostream>

using namespace Utilities;
using namespace GenParam;
using namespace std;

template <typename T>
bool GazeboSceneLoader::getSDFParameter(const sdf::ElementPtr sdf, T &parameter, const std::string &parameterName, const T &defaultValue)
{
	bool result = false;
	if (sdf->HasElement(parameterName))
	{
		result = true;
		parameter = sdf->GetElement(parameterName)->Get<T>();
	}
	else
	{
		parameter = defaultValue;
	}
	return result;
}

void GazeboSceneLoader::getVector3rParameter(const sdf::ElementPtr sdf, Vector3r &parameter, const ::std::string &parameterName, const Vector3r &defaultValue)
{
	if (sdf->HasElement(parameterName))
	{
		ignition::math::Vector3d value = sdf->GetElement(parameterName)->Get<ignition::math::Vector3d>();
		parameter = Vector3r(value.X(), value.Y(), value.Z());
	}
	else
	{
		parameter = defaultValue;
	}
}

/* void GazeboSceneLoader::getVector3iParameter(const sdf::ElementPtr sdf, Eigen::Matrix<unsigned int, 3, 1> &parameter, const ::std::string &parameterName, const Eigen::Matrix<unsigned int, 3, 1> &defaultValue)
{
	if (sdf->HasElement(parameterName))
	{
		ignition::math::Vector value = sdf->GetElement(parameterName)->Get<ignition::math::Vector3d>();
		parameter = Eigen::Matrix<unsigned int, 3, 1>(value.X(), value.Y(), value.Z());
	}
	else
	{
		parameter = defaultValue;
	}
} */

void GazeboSceneLoader::processFluidModels(Scene &scene, const sdf::ElementPtr &sdf)
{
	if (sdf->HasElement("fluidModels"))
	{
		sdf::ElementPtr fluidModelsElement = sdf->GetElement("fluidModels");
		while (fluidModelsElement)
		{
			std::string particleFile;
			getSDFParameter<std::string>(fluidModelsElement, particleFile, "particleFile", "");
			if (particleFile != "")
			{
				SceneLoader::FluidData *data = new SceneLoader::FluidData();
				data->samplesFile = particleFile;

				getSDFParameter<std::string>(fluidModelsElement, data->id, "id", "Fluid");
				getVector3rParameter(fluidModelsElement, data->translation, "translation", Vector3r::Zero());
				getVector3rParameter(fluidModelsElement, data->scale, "scale", Vector3r::Ones());

				data->rotation = Matrix3r::Identity();
				Vector3r rotationAxis;
				Real rotationAngle;
				getVector3rParameter(fluidModelsElement, rotationAxis, "rotationAxis", Vector3r(1.0, 0.0, 0.0));
				getSDFParameter<Real>(fluidModelsElement, rotationAngle, "rotationAngle", 0.0);
				data->rotation = AngleAxisr(rotationAngle, rotationAxis);

				getVector3rParameter(fluidModelsElement, data->initialVelocity, "initialVelocity", Vector3r::Zero());
				getSDFParameter<bool>(fluidModelsElement, data->invert, "invert", false);

				/* Eigen::Matrix<unsigned int, 3, 1> resolutionSDF;
				getVector3iParameter(fluidModelsElement,resolutionSDF ,"resolutionSDF", Eigen::Matrix<unsigned int, 3, 1>(20,20,20) );
				data->resolutionSDF[0] = resolutionSDF[0];
				data->resolutionSDF[1] = resolutionSDF[1];
				data->resolutionSDF[2] = resolutionSDF[2]; */

				getSDFParameter<unsigned char>(fluidModelsElement, data->mode, "denseMode", 0);

				scene.fluidModels.push_back(data);
			}
			fluidModelsElement = fluidModelsElement->GetNextElement("fluidModels");
		}
	}
}

void GazeboSceneLoader::processFluidBlocks(Scene &scene, const sdf::ElementPtr &sdf)
{
	if (sdf->HasElement("fluidBlock"))
	{
		sdf::ElementPtr fluidBlockElement = sdf->GetElement("fluidBlock");
		while (fluidBlockElement)
		{
			Vector3r translation, scale;
			getVector3rParameter(fluidBlockElement, translation, "translation", Vector3r::Zero());
			getVector3rParameter(fluidBlockElement, scale, "scale", Vector3r::Ones());

			Vector3r minX, maxX;
			getVector3rParameter(fluidBlockElement, minX, "start", Vector3r::Zero());
			getVector3rParameter(fluidBlockElement, maxX, "end", Vector3r::Ones());

			SceneLoader::FluidBlock *block = new SceneLoader::FluidBlock();
			block->box.m_minX[0] = scale[0] * minX[0] + translation[0];
			block->box.m_minX[1] = scale[1] * minX[1] + translation[1];
			block->box.m_minX[2] = scale[2] * minX[2] + translation[2];
			block->box.m_maxX[0] = scale[0] * maxX[0] + translation[0];
			block->box.m_maxX[1] = scale[1] * maxX[1] + translation[1];
			block->box.m_maxX[2] = scale[2] * maxX[2] + translation[2];

			getSDFParameter<std::string>(fluidBlockElement, block->id, "id", "Fluid");
			getSDFParameter<unsigned char>(fluidBlockElement, block->mode, "denseMode", 0);
			getVector3rParameter(fluidBlockElement, block->initialVelocity, "initialVelocity", Vector3r::Zero());

			scene.fluidBlocks.push_back(block);
			fluidBlockElement = fluidBlockElement->GetNextElement("fluidBlock");
		}
	}
}

void GazeboSceneLoader::processFluidEmmiters(Scene &scene, const sdf::ElementPtr &sdf)
{
	if (sdf->HasElement("fluidEmmiter"))
	{
		sdf::ElementPtr fluidEmmiterElement = sdf->GetElement("fluidEmmiter");
		while (fluidEmmiterElement)
		{
			SceneLoader::EmitterData *data = new SceneLoader::EmitterData();

			getSDFParameter<std::string>(fluidEmmiterElement, data->id, "id", "Fluid");
			getSDFParameter<unsigned int>(fluidEmmiterElement, data->width, "width", 5);
			getSDFParameter<unsigned int>(fluidEmmiterElement, data->height, "height", 5);
			getVector3rParameter(fluidEmmiterElement, data->x, "translation", Vector3r::Zero());

			Vector3r axis;
			Real angle;
			data->rotation = Matrix3r::Identity();
			getVector3rParameter(fluidEmmiterElement, axis, "rotationAxis", Vector3r(0, 0, 1));
			getSDFParameter<Real>(fluidEmmiterElement, angle, "rotationAngle", 0.0);
			axis.normalize();
			data->rotation = AngleAxisr(angle, axis).toRotationMatrix();

			getSDFParameter<Real>(fluidEmmiterElement, data->velocity, "velocity", 1.0);
			getSDFParameter<Real>(fluidEmmiterElement, data->emitStartTime, "emitStartTime", 0.0);
			getSDFParameter<Real>(fluidEmmiterElement, data->emitEndTime, "emitStartTime", std::numeric_limits<Real>::max());

			// type: 0 = rectangular, 1 = circle
			getSDFParameter<unsigned int>(fluidEmmiterElement, data->type, "type", 0);

			scene.emitters.push_back(data);
			fluidEmmiterElement = fluidEmmiterElement->GetNextElement("fluidEmmiter");
		}
	}
}

void GazeboSceneLoader::readScene(sdf::ElementPtr sdf, Scene &scene)
{
	LOG_INFO << "Load scene file from sdf: ";
	if (sdf->HasElement("fluidConfiguration"))
	{
		this->fluidConfiguration = sdf->GetElement("fluidConfiguration");
	}
	else
	{
		std::cout << "Fluid configuration missing from sdf, using default values" << std::endl;
	}

	getSDFParameter<Real>(this->fluidConfiguration, scene.timeStepSize, "timeStepSize", 0.001);
	getSDFParameter<Real>(this->fluidConfiguration, scene.particleRadius, "particleRadius", 0.025);

	processFluidModels(scene, sdf);
	processFluidBlocks(scene, sdf);
	processFluidEmmiters(scene, sdf);

	//LOG_INFO << "timeStepSize" << scene.timeStepSize << std::endl;
}

/* template <>
bool GazeboSceneLoader::readValue(const nlohmann::json &j, bool &v)
{
	if (j.is_null())
		return false;

	if (j.is_number_integer())
	{
		int val = j.get<int>();
		v = val != 0;
	}
	else
		v = j.get<bool>();
	return true;
} */

void GazeboSceneLoader::readParameterObject(const std::string &key, ParameterObject *paramObj)
{
	if (paramObj == nullptr)
		return;

	const unsigned int numParams = paramObj->numParameters();
	
	//////////////////////////////////////////////////////////////////////////
	// read configuration 
 	//////////////////////////////////////////////////////////////////////////
	if (this->fluidConfiguration->HasElement(key))
	{
		sdf::ElementPtr config = this->fluidConfiguration->GetElement(key);
		std::vector<std::string> newParamList;

		for (unsigned int i = 0; i < numParams; i++)
		{
			ParameterBase *paramBase = paramObj->getParameter(i);

			if (paramBase->getType() == RealParameterType)
			{
				Real val;
				if (getSDFParameter<Real>(config,val,paramBase->getName(),0.0 ))
				/* if (readValue(config[paramBase->getName()], val)) */
					static_cast<NumericParameter<Real>*>(paramBase)->setValue(val);
			}
			/* else if (paramBase->getType() == ParameterBase::UINT32)
			{
				unsigned int val;
				if (readValue(config[paramBase->getName()], val))
					static_cast<NumericParameter<unsigned int>*>(paramBase)->setValue(val);
			}
			else if (paramBase->getType() == ParameterBase::UINT16)
			{
				unsigned short val;
				if (readValue(config[paramBase->getName()], val))
					static_cast<NumericParameter<unsigned short>*>(paramBase)->setValue(val);
			}
			else if (paramBase->getType() == ParameterBase::UINT8)
			{
				unsigned char val;
				if (readValue(config[paramBase->getName()], val))
					static_cast<NumericParameter<unsigned char>*>(paramBase)->setValue(val);
			}
			else if (paramBase->getType() == ParameterBase::INT32)
			{
				int val;
				if (readValue(config[paramBase->getName()], val))
					static_cast<NumericParameter<int>*>(paramBase)->setValue(val);
			}
			else if (paramBase->getType() == ParameterBase::INT16)
			{
				short val;
				if (readValue(config[paramBase->getName()], val))
					static_cast<NumericParameter<short>*>(paramBase)->setValue(val);
			}
			else if (paramBase->getType() == ParameterBase::INT8)
			{
				char val;
				if (readValue(config[paramBase->getName()], val))
					static_cast<NumericParameter<char>*>(paramBase)->setValue(val);
			}
			else if (paramBase->getType() == ParameterBase::ENUM)
			{
				int val;
				if (readValue(config[paramBase->getName()], val))
					static_cast<EnumParameter*>(paramBase)->setValue(val);
			}
			else if (paramBase->getType() == ParameterBase::BOOL)
			{
				bool val;
				if (readValue(config[paramBase->getName()], val))
					static_cast<BoolParameter*>(paramBase)->setValue(val);
			}
			else if (paramBase->getType() == RealVectorParameterType)
			{
				if (static_cast<VectorParameter<Real>*>(paramBase)->getDim() == 3)
				{
					Vector3r val;
					if (readVector(config[paramBase->getName()], val))
						static_cast<VectorParameter<Real>*>(paramBase)->setValue(val.data());
				}
			}
			else if (paramBase->getType() == ParameterBase::STRING)
			{
				std::string val;
				if (readValue(config[paramBase->getName()], val))
					static_cast<StringParameter*>(paramBase)->setValue(val);
			} */
		}
	}
}