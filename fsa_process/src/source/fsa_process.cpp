/*******************************************************************************/
/* 
	Nombre: FSA_process
	Descripcion: Este proceso de Aerostack permite activar el algorimo de búsqueda
	de Fibonacci para obtener el valor óptimo de 2 de las ganancias de los
	controladores PID de altura y posición.
	Autor: Antonio Manuel García Estraviz
	
	This program is free software: you can redistribute it and/or modify
 	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program. If not, see http://www.gnu.org/licenses/.
*/
/*******************************************************************************/

#include "fsa_process.h"

//Constructor
FSA_process::FSA_process():RobotProcess()
{

}

//Destructor
FSA_process::~FSA_process()
{

}

/*******************************************************************************/
/*
	Nombre: void FSA_process::removeSpacesTabsLinebreaks(std::string& text)
	Descripcion: Elimina los saltos de línea, espacios y tabulaciones de una 
	cadena de texto en formato std::string
	Valor devuelto: void
	Autor: Antonio Manuel García Estraviz
*/
/*******************************************************************************/
void FSA_process::removeSpacesTabsLinebreaks(std::string& text)
{
  text.erase(std::remove_if(text.begin(), text.end(), isspace), text.end());
  text.erase(std::remove(text.begin(), text.end(), '\n'), text.end());
  text.erase(std::remove(text.begin(), text.end(), '\t'), text.end());
}

/*******************************************************************************/
/*
	Nombre: double FSA_process::readParameter(XMLFileReader& xmlFileReader, const std::string& parameterName, const double& defaultValue)
	Descripcion: Lee un parametro de un archivo XML con el nombre especificado,
	si no se encuentra el parametro, se devuelve el valor por defecto
*/
/*******************************************************************************/
double FSA_process::readParameter(XMLFileReader& xmlFileReader, const std::string& parameterName, const double& defaultValue)
{
  return xmlFileReader.readDoubleValue(parameterName);
}

/*******************************************************************************/
/*
	Nombre: int FSA_process::readParameter(XMLFileReader& xmlFileReader, const std::string& parameterName, const double& defaultValue)
	Descripcion: Lee un parametro de un archivo XML con el nombre especificado,
	si no se encuentra el parametro, se devuelve el valor por defecto
*/
/*******************************************************************************/
int FSA_process::readParameter(XMLFileReader& xmlFileReader, const std::string& parameterName, const int& defaultValue)
{
  return xmlFileReader.readIntValue(parameterName);
}

/*******************************************************************************/
/*
	Nombre: std::string FSA_process::readParameter(XMLFileReader& xmlFileReader, const std::string& parameterName, const double& defaultValue)
	Descripcion: Lee un parametro de un archivo XML con el nombre especificado,
	si no se encuentra el parametro, se devuelve el valor por defecto
*/
/*******************************************************************************/
std::string FSA_process::readParameter(XMLFileReader& xmlFileReader, const std::string& parameterName, const std::string defaultValue)
{
  return xmlFileReader.readStringValue(parameterName);
}

/*******************************************************************************/
/*
	Nombre: template<typename T> T FSA_process::readXMLParameter(XMLFileReader &xmlFileReader, const std::string& parameterName, T defaultValue)
	Descripcion: Lee un parametro de un archivo XML con el nombre especificado,
	si no se encuentra el parametro, se devuelve el valor por defecto. 
	Imprime por pantalla el resultado de la lectura
*/
/*******************************************************************************/
template<typename T> T FSA_process::readXMLParameter(XMLFileReader &xmlFileReader, const std::string& parameterName, T defaultValue)
{
  T parameterValue;
  try
  {
    parameterValue = readParameter(xmlFileReader, parameterName, defaultValue);
    std::cout << "Parameter \"" << BOLDWHITE << parameterName << RESET << "\""<< GREEN <<" Found "<< RESET << "Value: " << BOLDWHITE << parameterValue << RESET << std::endl;
  }
  catch ( cvg_XMLFileReader_exception &e)
  {
    parameterValue = defaultValue;
    std::cout << "Parameter \"" << BOLDWHITE << parameterName << RESET << "\"" << RED <<" Not found "<< RESET << "Default value is used. Value: " << BOLDWHITE << parameterValue << RESET << std::endl;
  }
  return parameterValue;
}

/*******************************************************************************/
/*
	Nombre: bool FSA_process::readConfigs(const std::string& configFile)
	Descripcion: Lee la configuracion a utilizar durante el proceso de ajuste 
	desde un archivo XML
	Muestra en pantalla si el proceso de lectura ha sido efectuado con éxito.
	En caso contrario, se utilizaran los valores por defecto
*/
/*******************************************************************************/
bool FSA_process::readConfigs(const std::string& configFile)
{
  try
  {
    std::cout << BLUE << "Reading configuration variables from config.xml..." << RESET << std::endl;
    XMLFileReader xmlFileReader(configFile);

    parInitial = readXMLParameter(xmlFileReader, "gains:gain2init", 0.5);
    p1Range[LEFT] = readXMLParameter(xmlFileReader, "gains:gain1:range:min", 0);
    p1Range[RIGHT] = readXMLParameter(xmlFileReader, "gains:gain1:range:max", 5);
    p2Range[LEFT] = readXMLParameter(xmlFileReader, "gains:gain2:range:min", 0);
    p2Range[RIGHT] = readXMLParameter(xmlFileReader, "gains:gain2:range:max", 5);
    gain1type = readXMLParameter(xmlFileReader, "gains:gain1:type", 1);
    gain2type = readXMLParameter(xmlFileReader, "gains:gain2:type", 2);
    numberOfTests = readXMLParameter(xmlFileReader, "numTests", 1);

    std::string controller = readXMLParameter(xmlFileReader, "controller", std::string("altitude"));
    removeSpacesTabsLinebreaks(controller);
    if(controller == "altitude")
    {
      reconfigureGainsFunction = &MyProcess::reconfigureAltitudeGains;
      gain_reconfigure_topic_name = altitude_gain_reconfigure_topic_name;
      costFunction = &MyProcess::altitudeCostFunction;
    }
    else if(controller == "position")
    {
      reconfigureGainsFunction = &MyProcess::reconfigurePositionGains;
      gain_reconfigure_topic_name = position_gain_reconfigure_topic_name;
      costFunction = &MyProcess::positionCostFunction;
    }
    else
    {
      reconfigureGainsFunction = &MyProcess::reconfigureAltitudeGains;
      gain_reconfigure_topic_name = altitude_gain_reconfigure_topic_name;
      costFunction = &MyProcess::altitudeCostFunction;
    }

    std::string referenceAxis = readXMLParameter(xmlFileReader, "referenceAxis", std::string("z"));
    removeSpacesTabsLinebreaks(referenceAxis);
    float z = offset;
    if(referenceAxis == "x")
    {
      initialReference[0] = offset;
      initialReference[1] = 0.0;
      initialReference[2] = z;

      finalReference[0] = offset + scale;
      finalReference[1] = 0.0;
      finalReference[2] = z;
    }
    else if(referenceAxis == "y")
    {
      initialReference[0] = 0.0;
      initialReference[1] = offset;
      initialReference[2] = z;

      finalReference[0] = 0.0;
      finalReference[1] = offset + scale;
      finalReference[2] = z;
    }
    else if(referenceAxis == "z")
    {
      initialReference[0] = 0.0;
      initialReference[1] = 0.0;
      initialReference[2] = offset;

      finalReference[0] = 0.0;
      finalReference[1] = 0.0;
      finalReference[2] = offset + scale;
    }
    else
    {
      initialReference[0] = 0.0;
      initialReference[1] = 0.0;
      initialReference[2] = offset;

      finalReference[0] = 0.0;
      finalReference[1] = 0.0;
      finalReference[2] = offset + scale;
    }
    std::cout << referenceAxis << std::endl;


  }
  catch ( cvg_XMLFileReader_exception &e)
  {
    std::cout << RED << "Error reading configuration file. Setting default values." << RESET << std::endl;

  }
}

/*******************************************************************************/
/*
	Nombre: void FSA_process::reconfigureAltitudeGains(float gain1, float gain2)
	Descripcion: Envía las ganancias al MidLevelController para que sean
	sustituidas por las especificadas en el controlador de altura
*/
/*******************************************************************************/
void FSA_process::reconfigureAltitudeGains(float gain1, float gain2)
{
  float gains[3];
  //Set default values for gains // CHANGE THIS (MOVE TO .H)
  gains[kp] = -1.33;
  gains[ki] = 0.0;
  gains[kd] = -0.35;

  //Overwrite the default gains with the new gains
  gains[gain1type] = -gain1;
  gains[gain2type] = -gain2;

  //Convert gains into a string and publish it
  std_msgs::String msg;
  std::stringstream ss;
  ss << std::to_string(gains[kp]) << " " << std::to_string(gains[ki]) << " " << std::to_string(gains[kd]);
  msg.data = ss.str();
  gain_reconfigure_pub.publish(msg);
}

/*******************************************************************************/
/*
	Nombre: void FSA_process::reconfigurePositionGains(float gain1, float gain2)
	Descripcion: Envía las ganancias al MidLevelController para que sean
	sustituidas por las especificadas en el controlador de posicion
*/
/*******************************************************************************/
void FSA_process::reconfigurePositionGains(float gain1, float gain2)
{
  float gains[3];
  //Set default values for gains // CHANGE THIS (MOVE TO .H)
  gains[kp] = 2.04;
  gains[ki] = 0.10;
  gains[kd] = 0.99;

  //Overwrite the default gains with the new gains
  gains[gain1type] = gain1;
  gains[gain2type] = gain2;

  //Convert gains into a string and publish it
  std_msgs::String msg;
  std::stringstream ss;
  ss << std::to_string(gains[kp]) << " " << std::to_string(gains[ki]) << " " << std::to_string(gains[kd]);
  msg.data = ss.str();
  gain_reconfigure_pub.publish(msg);
}


/*******************************************************************************/
/*
	Nombre: void FSA_process::sendReference(float reference[3])
	Descripcion: Envía la posición y ángulo de yaw de referencia
*/
/*******************************************************************************/
void FSA_process::sendReference(float reference[3])
{
  droneMsgsROS::dronePositionRefCommandStamped pose_msg;
  pose_msg.position_command.x = reference[0];
  pose_msg.position_command.y = reference[1];
  pose_msg.position_command.z = reference[2];
  drone_position_ref_pub.publish(pose_msg);

  droneMsgsROS::droneYawRefCommand yaw_msg;
  yaw_msg.yaw = 0.0;
  drone_yaw_ref_pub.publish(yaw_msg);
}

/*******************************************************************************/
/*
	Nombre: double FSA_process::altitudeCostFunction()
	Descripcion: Calcula el indice de rendimiento como la diferencia en valor
	absoluto de la altura del drone y la trayectoria de referencia
*/
/*******************************************************************************/
double FSA_process::altitudeCostFunction()
{
  return std::abs((trajectoryPrimitive[NcCounter]*scale + offset) - currentZ);
}

/*******************************************************************************/
/*
	Nombre: double FSA_process::positionCostFunction()
	Descripcion: Calcula el indice de rendimiento como la diferencia en valor
	absoluto de la posicion del drone y la trayectoria de referencia
*/
/*******************************************************************************/
double FSA_process::positionCostFunction()
{
  return std::abs((trajectoryPrimitive[NcCounter]*scale + offset) - currentX)+
      std::abs(currentY) +
      std::abs(offset - currentZ);
}

/*******************************************************************************/
/*
	Nombre: void FSA_process::createFile(const std::string &path, const std::string &filename)
	Descripcion: Crea un archivo de guardado para almacenar información acerca
	del proceso de ajuste
*/
/*******************************************************************************/
void FSA_process::createFile(const std::string &path, const std::string &filename)
{
  std::ofstream outfile(path + "/" + filename + std::to_string(testCounter) + ".txt");
  outfile.close();
  if (outfile.fail()) {
    std::cout << RED << "Could not create file at: " << path << RESET << std::endl;
  }
}

/*******************************************************************************/
/*
	Nombre: void FSA_process::saveDataToFile(const std::string &path, const std::string &filename, std::string &data)
	Descripcion: Guarda la información especificada en formato std::string en 
	la ruta especificada
*/
/*******************************************************************************/
void FSA_process::saveDataToFile(const std::string &path, const std::string &filename, std::string &data)
{
  std::ofstream myfile;
  myfile.open (path + "/" + filename + std::to_string(testCounter) + ".txt", std::ios_base::app);
  myfile << data;
  myfile.close();
  if (myfile.fail())
  {
    std::cout << RED << "Could not save data at: " << path << RESET << std::endl;
  }
  data = "";
}

/*******************************************************************************/
/*
	Nombre: double FSA_process::FibonacciNumber(int n)
	Descripcion: Calcula el n número de fibonacci 
*/
/*******************************************************************************/
double FSA_process::FibonacciNumber(int n)
{
  int FFF;
  int F;
  int FF;
  int i;
  FFF = 0;
  if (n > 0)
  {
    F = 0;
    FF = 1;
    for (i = 0; i < n; i++)
    {
      FFF = FF + F;
      F = FF;
      FF = FFF;
    }
  }
  else
  {
    FFF = 1;
  }
  return FFF;
}


/*******************************************************************************/
/*
	Nombre: void FSA_process::generateFibonacciRange(double i, double N_max, const double current_range[2], double new_range[2])
	Descripcion: Genera un rango de Fibonacci a partir del rango anterior
	y el número de iteracion
*/
/*******************************************************************************/
void FSA_process::generateFibonacciRange(double i, double N_max, const double current_range[2], double new_range[2])
{
  double rho;
  double length_old;
  double dd;
  rho = FibonacciNumber(N_max - i) / FibonacciNumber((N_max - i) + 2.0);
  if (i == N_max)
  {
    dd = 0.01;
  }
  else
  {
    dd = 0.0;
  }

  /*  previous range */
  length_old = current_range[1] - current_range[0];
  new_range[0] = current_range[0] + (rho - dd) * length_old;
  new_range[1] = current_range[0] + ((1.0 - rho) + dd) * length_old;
}

/*******************************************************************************/
/*
	Nombre: void FSA_process::initFibonacciVariables()
	Descripcion: Inicializa los valores de las variables del algoritmo de 
	Fibonacci para poder comenzar el proceso.
*/
/*******************************************************************************/
void FSA_process::initFibonacciVariables()
{
  //Reset Fibonacci Search Algorithm Variables
  iterationCounter = 0;
  NcCounter = 0;
  J = 0;
  par = 0;
  delayCounter = 0;
  iterationCounterPerCycle = 0;

  //Init Fibonacci Search Algorithm
  previousRange[LEFT] = p1Range[LEFT];
  previousRange[RIGHT] = p1Range[RIGHT];

  //Generate range for P1
  generateFibonacciRange(1, numRanges, p1Range, newRange);

  params[P1] = newRange[LEFT];
  params[P2] = parInitial;

  (this->*reconfigureGainsFunction)(params[P1], params[P2]);

  firstIteration = true;
  running = true;
}



/*******************************************************************************/
/*
	Nombre: void FSA_process::ownSetUp()
	Descripcion: Lee los archivos de configuración
*/
/*******************************************************************************/
void FSA_process::ownSetUp()
{
  readConfigs(ros::package::getPath("my_process") + "/config.xml");

}

/*******************************************************************************/
/*
	Nombre: void FSA_process::ownStart()
	Descripcion: Comienza el método de ajuste. Esta función es ejecutada
	cuando el proceso es iniciado
*/
/*******************************************************************************/
void FSA_process::ownStart()
{
  //!Init communications
  //Init publishers
  gain_reconfigure_pub = node_handle.advertise<std_msgs::String>(gain_reconfigure_topic_name, 1);
  drone_position_ref_pub = node_handle.advertise<droneMsgsROS::dronePositionRefCommandStamped>(drone_position_ref_topic_name,1);
  drone_yaw_ref_pub = node_handle.advertise<droneMsgsROS::droneYawRefCommand>(drone_yaw_ref_pub_name, 1);

  //Init subscribers
  estimated_pose_sub = node_handle.subscribe(estimated_pose_topic_name, 1, &MyProcess::estimatedPoseCallBack, this);
  estimated_speeds_sub = node_handle.subscribe(estimated_speeds_topic_name, 1, &MyProcess::estimatedSpeedsCallBack, this);

  //Register starting time
  start_time = std::chrono::steady_clock::now();

  //Init fibonacci variables
  initFibonacciVariables();

  //Print variables
  std::cout << BOLDWHITE <<"Running test #" << std::to_string(testCounter) << " of " << numberOfTests << " tests " << RESET << std::endl;
  std::cout << std::fixed;
  std::cout << std::setprecision(4);
  std::cout << "Iter." << "	" << "Par" << "	" << "Par(-)" << "	" << "Par(+)" << "	" << "Par1" << "	" << "Par2" << "	" << "J" << std::endl;
  std::cout << "-------------------------------------------------------" << std::endl;

  //Create files for saving simulation data
  createFile(drone_pose_recording_path, drone_pose_recording_filename);
  createFile(gains_recording_path, gains_recording_filename);

}

/*******************************************************************************/
/*
	Nombre: void FSA_process::ownStop()
	Descripcion: 
*/
/*******************************************************************************/
void FSA_process::ownStop()
{

}

/*******************************************************************************/
/*
	Nombre: void FSA_process::ownRun()
	Descripcion: Ejecución del sistema de ajuste de Fibonacci. Es ejecutado 
	de forma continua a una frecuencia especificada en el archivo main del 
	proceso.
*/
/*******************************************************************************/
void FSA_process::ownRun()
{
  drone_pose_recording.append(
        std::to_string(currentX) + " " +
        std::to_string(currentY) + " " +
        std::to_string(currentZ) + " " +
        std::to_string(J) + "\n");

  if(running){
    if (firstIteration)
    {
      //First Iteration
      delayCounter++;
      if(delayCounter == initialDelay/2)
      {
        sendReference(initialReference);
      }
      else if(delayCounter >= initialDelay)
      {
        sendReference(finalReference);
        firstIteration = false;
      }
    }
    else
    {
      if (NcCounter < Nc)
      {
        //Collect data for performance index

        mutex_position.lock();
        {
          J += (this->*costFunction)();//std::abs((trajectoryPrimitive[NcCounter]*scale + offset) - currentZ);
        }
        mutex_position.unlock();
        if(NcCounter == 25)
        {
          sendReference(initialReference);
        }
      }
      else if (NcCounter == Nc)
      {
        //Initialize fibonacci iteration
        iterationCounter++;
        iterationCounterPerCycle++;

        //If the iteration counter is odd, evaluate left bound
        if (iterationCounter % 2 != 0)
        {
          Js[LEFT] = J;

          //Print status
          std::cout << iterationCounter << "	" << par + 1 << "	" << previousRange[LEFT] << "	" << previousRange[RIGHT] << "	" << params[P1] << "	" << params[P2] << "	" << Js[LEFT]/Nc << std::endl;
          gains_recording.append(
                std::to_string(iterationCounter) + " " +
                std::to_string(par + 1) + " " +
                std::to_string(previousRange[LEFT]) + " " +
                std::to_string(previousRange[RIGHT]) + " " +
                std::to_string(params[P1]) + " " +
                std::to_string(params[P2]) + " " +
                std::to_string(Js[LEFT]/Nc) + "\n");

          //The next parameter is the one refered to the right bound
          params[par] = newRange[RIGHT];

        }
        //If the iteration counter is even, evalutate right bound and calculate new range
        else
        {
          //Store final value for performance index for P2
          Js[RIGHT] = J;

          //Print status
          std::cout << iterationCounter << "	" << par + 1 << "	" << previousRange[LEFT] << "	" << previousRange[RIGHT] << "	" << params[P1] << "	" << params[P2] << "	" << Js[RIGHT]/Nc << std::endl;
          gains_recording.append(
                std::to_string(iterationCounter) + " " +
                std::to_string(par + 1) + " " +
                std::to_string(previousRange[LEFT]) + " " +
                std::to_string(previousRange[RIGHT]) + " " +
                std::to_string(params[P1]) + " " +
                std::to_string(params[P2]) + " " +
                std::to_string(Js[RIGHT]/Nc) + "\n");

          //Check if it is not the last iteration for this parameter
          if (!(iterationCounter % (numRanges * 2) == 0))
          {
            //If the value of the left bound is smaller than the value of the right bound. Reduce range
            if (Js[LEFT] < Js[RIGHT])
            {
              previousRange[RIGHT] = newRange[RIGHT];
            }
            //If the value of the right bound is smaller than the value of the left bound. Reduce range
            else
            {
              previousRange[LEFT] = newRange[LEFT];
            }

            //Calculate new range
            generateFibonacciRange(iterationCounterPerCycle / 2 + 1, numRanges, previousRange, newRange);

            //The next parameter is the one refered to the left bound
            params[par] = newRange[LEFT];
          }
          //Check if it is the last iteration for this parameter
          else if(iterationCounter < numIterations)
          {
            //If parameter evalueted was P1, then change to P2 and restart the process
            if (par == P1)
            {
              par = P2;

              params[P1] = (params[P1] + previousRange[LEFT]) / 2;//Approximation. Check how to calculate this. IMPORTANT

              previousRange[LEFT] = p2Range[LEFT];
              previousRange[RIGHT] = p2Range[RIGHT];
              generateFibonacciRange(1, numRanges, p2Range, newRange);
              iterationCounterPerCycle = 0;

              params[P2] = newRange[LEFT];
              std::cout << "-------------------------------------------------------" << std::endl;
            }
            //If parameter evalueted was P2, then change to P1 and restart the process
            else
            {
              par = P1;
              params[P2] = (params[P2] + previousRange[LEFT]) / 2; //Approximation. Check how to calculate this. IMPORTANT

              previousRange[LEFT] = p1Range[LEFT];
              previousRange[RIGHT] = p1Range[RIGHT];
              generateFibonacciRange(1, numRanges, p1Range, newRange);
              iterationCounterPerCycle = 0;

              params[P1] = newRange[LEFT];

              std::cout << "-------------------------------------------------------" << std::endl;
            }
          }
        }

        //Check if do this here?
        (this->*reconfigureGainsFunction)(params[P1], params[P2]);
        //sendReference(offset);
      }
      else if (NcCounter < NcMax)
      {
        //Transitient decay for waiting the change of the controller parameters

      }
      else
      {
        //End of the transitient decay
        //Reset values
        NcCounter = -1;//This should be 0!!!
        J = 0;
        sendReference(finalReference);

        saveDataToFile(drone_pose_recording_path, drone_pose_recording_filename, drone_pose_recording);
        saveDataToFile(gains_recording_path, gains_recording_filename, gains_recording);
      }

      NcCounter++;

      //Stop the algorithm when the searching has finished
      if (iterationCounter >= numIterations)
      {
        saveDataToFile(drone_pose_recording_path, drone_pose_recording_filename, drone_pose_recording);
        saveDataToFile(gains_recording_path, gains_recording_filename, gains_recording);

        if(par == P1)
        {
          params[P1] = (params[P1] + previousRange[LEFT]) / 2;
        }
        else
        {
          params[P2] = (params[P2] + previousRange[LEFT]) / 2;
        }

        std::cout << BOLDGREEN << "Tunning Finished" << RESET << std::endl;
        auto end_time = std::chrono::steady_clock::now();
        std::cout << "Elapsed time in seconds : "
                  << std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time).count()
                  << " sec" << std::endl;

        running = false;

        std::cout << "Parameters obtained p1: " << BOLDWHITE << params[P1]
                     << RESET <<" p2: " << BOLDWHITE << params[P2] << RESET << std::endl;

        //Prepare for next test //IMPROVE THIS
        testCounter++;
        if(testCounter <= numberOfTests)
        {
          start_time = std::chrono::steady_clock::now();

          initFibonacciVariables();

          std::cout << BOLDWHITE <<"Running test #" << testCounter << " of " << numberOfTests << " tests " << RESET << std::endl;

          //Print variables
          std::cout << std::fixed;
          std::cout << std::setprecision(4);
          std::cout << "Iter." << "	" << "Par" << "	" << "Par(-)" << "	" << "Par(+)" << "	" << "Par1" << "	" << "Par2" << "	" << "J" << std::endl;
          std::cout << "-------------------------------------------------------" << std::endl;

          createFile(drone_pose_recording_path, drone_pose_recording_filename);
          createFile(gains_recording_path, gains_recording_filename);
        }
      }
    }
  }
}

/*******************************************************************************/
/*
	Nombre: void FSA_process::estimatedPoseCallBack(const droneMsgsROS::dronePose& message)
	Descripcion: Callback que almacena la posición del UAV 
*/
/*******************************************************************************/
void FSA_process::estimatedPoseCallBack(const droneMsgsROS::dronePose& message)
{
  mutex_position.lock();
  {
    positionRecieved = true;
    currentZ = -message.z; ///////CHANGE THIS IN BEBOP FLIGHT????
    currentX = message.x;
    currentY = message.y;
  }
  mutex_position.unlock();
}

/*******************************************************************************/
/*
	Nombre: void FSA_process::estimatedPoseCallBack(const droneMsgsROS::dronePose& message)
	Descripcion: Callback que almacena la velocidad del UAV
*/
/*******************************************************************************/
void FSA_process::estimatedSpeedsCallBack(const droneMsgsROS::droneSpeeds& message)
{
  mutex_speed.lock();
  {
    currentSpeed = message.dz;
  }
  mutex_speed.unlock();

}


/*******************************************************************************/
/*
	Nombre: void FSA_process::estimatedPoseCallBack(const droneMsgsROS::dronePose& message)
	Descripcion: Callback que almacena la posición del UAV 
*/
/*******************************************************************************/
void FSA_process::estimatedPoseCallBack(const droneMsgsROS::dronePose& message)
{
  mutex_position.lock();
  {
    positionRecieved = true;
    currentZ = -message.z; ///////CHANGE THIS IN BEBOP FLIGHT????
    currentX = message.x;
    currentY = message.y;
  }
  mutex_position.unlock();
}

/*******************************************************************************/
/*
	Nombre: void FSA_process::estimatedPoseCallBack(const droneMsgsROS::dronePose& message)
	Descripcion: Callback que almacena la velocidad del UAV
*/
/*******************************************************************************/
void FSA_process::estimatedSpeedsCallBack(const droneMsgsROS::droneSpeeds& message)
{
  mutex_speed.lock();
  {
    currentSpeed = message.dz;
  }
  mutex_speed.unlock();
}

