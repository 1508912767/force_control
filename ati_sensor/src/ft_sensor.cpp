#include "ft_sensor.h"
#include <stdexcept>
#include <sys/socket.h>

// XML 相关的库
#include <libxml/parser.h>
#include <libxml/tree.h>
#include <libxml/encoding.h>
#include <libxml/xmlwriter.h>
#include <sstream>
#include <vector>
#include <string>

#define rt_dev_socket       socket
#define rt_dev_setsockopt   setsockopt
#define rt_dev_bind         bind
#define rt_dev_recvfrom     recvfrom
#define rt_dev_sendto       sendto
#define rt_dev_close        close
#define rt_dev_connect      connect
#define rt_dev_recv         recv
#define rt_dev_send         send
#define RT_SO_TIMEOUT       SO_RCVTIMEO

static std::string getStringInXml(const std::string& xml_s,const std::string& tag)
{
    const std::string tag_open = "<"+tag+">";
    const std::string tag_close = "</"+tag+">";
    const std::size_t n_start = xml_s.find(tag_open);
    const std::size_t n_end = xml_s.find(tag_close);
    return xml_s.substr(n_start+tag_open.length(),n_end);
}
template<typename T>
static T getNumberInXml(const std::string& xml_s,const std::string& tag)
{
    const std::string num = getStringInXml(xml_s,tag);
    double r = ::atof(num.c_str());
    return static_cast<T>(r);
}
template<typename T>
static bool getArrayFromString(const std::string& str,const char delim,T *data,size_t len)
{
    size_t start = str.find_first_not_of(delim), end=start;
    size_t idx = 0;
    while (start != std::string::npos && idx < len){
        end = str.find(delim, start);
        std::string token = str.substr(start, end-start);
        if (token.empty())
          token = "0.0";
        double r = ::atof(token.c_str());
        data[idx] = static_cast<T>(r);
        ++idx;
        start = str.find_first_not_of(delim, end);
    }
    return (idx == len);
}
template<typename T>
static bool getArrayFromXml(const std::string& xml_s,const std::string& tag,const char delim,T *data,size_t len)
{
    const std::string str = getStringInXml(xml_s,tag);
    return getArrayFromString<T>(str,delim,data,len);
}


using namespace ati;

FTSensor::FTSensor()
{
    // 初始化参数
    initialized_                = false;
    ip                          = ati::default_ip;
    port                        = command_s::DEFAULT_PORT;
    cmd_.command                = command_s::STOP;
    cmd_.sample_count           = 1;
    calibration_index           = ati::current_calibration;
    socketHandle_               = -1;
    resp_.cpf                   = 1000000;
    resp_.cpt                   = 1000000;
    rdt_rate_                   = 0;
    timeval_.tv_sec             = 2;
    timeval_.tv_usec            = 0;
    xml_s_.reserve(MAX_XML_SIZE);
    setbias_ = new int[6];
}

FTSensor::~FTSensor()
{
  stopStreaming();
  if(!closeSockets())
    std::cerr << message_header() << "Sensor did not shutdown correctly" << std::endl;
  delete setbias_;
}

bool FTSensor::startStreaming(int nb_samples)
{
  if (nb_samples < 0) {
    // use default sample_count
    return startStreaming();
  }
  else {
    // use given sample count
    uint32_t sample_count = static_cast<uint32_t>(nb_samples);
      switch(cmd_.command){
        case command_s::REALTIME:
    //std::cout << "Starting realtime streaming" << std::endl;
    return startRealTimeStreaming(sample_count);
        case command_s::BUFFERED:
    //std::cout << "Starting buffered streaming" << std::endl;
    return startBufferedStreaming(sample_count);
        case command_s::MULTIUNIT:
    //std::cout << "Starting multi-unit streaming" << std::endl;
    return startMultiUnitStreaming(sample_count);
        default:
    std::cout <<  message_header() << cmd_.command << ": command mode not allowed" << std::endl;
    return false;
      }
  }
}

// Initialization read from XML file
bool FTSensor::startStreaming()
{
    switch(cmd_.command){
      case command_s::REALTIME:
	//std::cout << "Starting realtime streaming" << std::endl;
	return startRealTimeStreaming();
      case command_s::BUFFERED:
	//std::cout << "Starting buffered streaming" << std::endl;
	return startBufferedStreaming();
      case command_s::MULTIUNIT:
	//std::cout << "Starting multi-unit streaming" << std::endl;
	return startMultiUnitStreaming();
      default:
	std::cout << message_header() << cmd_.command<< ": command mode not allowed" << std::endl;
	return false;
    }
}

bool FTSensor::init(std::string ip, int calibration_index, uint16_t cmd, int sample_count)
{
  //  Re-Initialize parameters
  initialized_ = true;
  this->ip = ip;
  this->port = command_s::DEFAULT_PORT;
  cmd_.command = command_s::STOP;
  cmd_.sample_count = 1;
  this->calibration_index = calibration_index;

  //  Open Socket
  if(!ip.empty() && openSockets())
  {
    if(!stopStreaming()) // if previously launched
        std::cerr << "\033[33m" << message_header() << "Could not stop streaming\033[0m" << std::endl;
    setCommand(cmd); // Setting cmd mode

    initialized_ &= startStreaming(sample_count);            // Starting streaming

    if (!initialized_)
    {
        std::cerr << "\033[1;31m" << message_header() << "Could not start streaming\033[0m" << std::endl;
        return initialized_;
    }
    initialized_ &= getResponse();

    // Parse Calibration from web server
    if(initialized_)
        getCalibrationData();
  }else
    initialized_ = false;

  if (!initialized_)
    std::cerr << "\033[1;31m" << message_header() << "Error during initialization, FT sensor NOT started\033[0m" << std::endl;

  return initialized_;
}
bool FTSensor::openSockets()
{
  try{
    // To get the online configuration (need to build rtnet with TCP option)
    openSocket(socketHTTPHandle_,getIP(),80,IPPROTO_TCP);
    // The data socket
    openSocket(socketHandle_,getIP(),getPort(),IPPROTO_UDP);
  }
  catch (std::exception &ex) {
    std::cerr << "\033[1;31m" << message_header() <<  "openSockets error: " << ex.what()  <<"\033[0m" << std::endl;
    return false;
  }
  return true;
}
void FTSensor::openSocket(int& handle,const std::string ip,const uint16_t port,const int option)
{
  // create the socket
    if (handle != -1)
        rt_dev_close(handle);
    std::string proto = "";
    if(option == IPPROTO_UDP)
    {
        proto = "UDP";
        handle = rt_dev_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    }
    else if(option == IPPROTO_TCP)
    {
        proto = "TCP";
        handle = rt_dev_socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    }
    else
        handle = rt_dev_socket(AF_INET, SOCK_DGRAM, 0);

    if (handle < 0) {
        std::cerr << "\033[31m" << message_header() << "Could not init sensor socket for proto [" << proto 
                  << "], please make sure your can ping the sensor\033[0m" << std::endl;
        throw std::runtime_error("failed to init sensor socket");
    }

    // re-use address in case it's still binded
    rt_dev_setsockopt(handle, SOL_SOCKET, SO_REUSEADDR, 0, 0);

    // set the socket parameters
    struct sockaddr_in addr = {0};
    hostent * hePtr = NULL;
    hePtr = gethostbyname(ip.c_str());
    memcpy(&addr.sin_addr, hePtr->h_addr_list[0], hePtr->h_length);

    //addr_.sin_addr.s_addr = INADDR_ANY;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);

    // connect
    if (rt_dev_connect(handle, (struct sockaddr*) &addr, sizeof(addr)) < 0)
    {
        std::cerr << "\033[31m" << message_header() << "Could not connect to " << ip << "\033[0m" << std::endl ;
        throw std::runtime_error("failed to connect to socket" );
    }
    return;
}
bool FTSensor::closeSockets()
{
  return closeSocket(socketHandle_) == 0 && closeSocket(socketHTTPHandle_) == 0;
}
int FTSensor::closeSocket(const int& handle)
{
  if(handle < 0)
    return 0;  // closing a non-open socket is considered successful
  return rt_dev_close(handle);
}

bool FTSensor::getCalibrationData()
{
  FTSensor::settings_error_t err = getSettings();
  if(err!=SETTINGS_REQUEST_ERROR && err!=CALIB_PARSE_ERROR)
  {
      std::cout << message_header() << "Sucessfully retrieved counts per force : " << resp_.cpf << std::endl;
      std::cout << message_header() << "Sucessfully retrieved counts per torque : " << resp_.cpt << std::endl;
  }
  else
  {
      std::cerr << message_header() << "Using default counts per force : " << resp_.cpf << std::endl;
      std::cerr << message_header() << "Using default counts per torque : " << resp_.cpt << std::endl;
  }
}

FTSensor::settings_error_t FTSensor::getSettings()
{
  std::string index("");
  if(calibration_index != ati::current_calibration)
  {
    std::stringstream ss;
    ss << calibration_index;
    index = "?index=" + ss.str();
    std::cout << message_header() << "Using calibration index "<<calibration_index<< std::endl;
  }else
      std::cout << message_header() << "Using current calibration" << std::endl;

    static const uint32_t chunkSize = 4;        // Every chunk of data will be of this size
    static const uint32_t maxSize = 65536;      // The maximum file size to receive
                          // The recv buffer
    std::string filename = "/netftapi2.xml"+index; // the name of the file to reveice
    std::string host = getIP();

    std::string request_s = "GET "+filename+" HTTP/1.1\r\nHost: "+host+"\r\n\r\n";

    if (rt_dev_send(socketHTTPHandle_, request_s.c_str(),request_s.length(), 0) < 0)
    {
        std::cerr << message_header() << "Could not send GET request to " << getIP()
                  << ":80. Please make sure that RTnet TCP protocol is installed" << std::endl;
        return SETTINGS_REQUEST_ERROR;
    }

    int recvLength=0;
    int posBuff = 0;
    while(posBuff < maxSize) // Just a security to avoid infinity loop
    {
            recvLength = rt_dev_recv(socketHTTPHandle_, &xml_c_[posBuff],chunkSize, 0);
            posBuff += recvLength;
            if(recvLength <= 0) // The last chunk returns 0
                break;
    }
    xml_s_ = xml_c_;

    const uint32_t cfgcpf_r = getNumberInXml<uint32_t>(xml_s_,"cfgcpf");
    const uint32_t cfgcpt_r = getNumberInXml<uint32_t>(xml_s_,"cfgcpt");
    const int cfgcomrdtrate = getNumberInXml<int>(xml_s_,"comrdtrate");
    rdt_rate_ = cfgcomrdtrate;

    // 6 tokens separated by semi-colon
    if (!getArrayFromXml<int>(xml_s_,"setbias",';',setbias_, 6))
    {
        return GAUGE_PARSE_ERROR;
    }

    if(cfgcpf_r && cfgcpt_r)
    {
        resp_.cpf = cfgcpf_r;
        resp_.cpt = cfgcpt_r;
        return NO_SETTINGS_ERROR;
    }
  std::cerr << message_header() << "Could not parse file " << filename << std::endl;
  return SETTINGS_REQUEST_ERROR;
}

bool FTSensor::sendTCPrequest(std::string &request_cmd)
{
  if (request_cmd.empty() )
  {
    std::cerr << message_header() << "Empty TCP command, not sending" << std::endl;
    return false;
  }
  else
  {
    static const uint32_t chunkSize = 4;        // Every chunk of data will be of this size
    static const uint32_t maxSize = 65536;      // The maximum file size to receive
    std::string host = getIP();

    std::string request_s = "GET "+request_cmd+" HTTP/1.0\r\nHost: "+host+"\r\n\r\n";

    if (rt_dev_send(socketHTTPHandle_, request_s.c_str(),request_s.length(), 0) < 0)
    {
        std::cerr << message_header() << "Could not send GET request to " << host
                  << ":80. Please make sure that RTnet TCP protocol is installed" << std::endl;
        return false;
    }

    //empty the buffer but we don't care about the result
    int recvLength=0;
    int posBuff = 0;
    while(posBuff < maxSize) // Just a security to avoid infinity loop
    {
        recvLength = rt_dev_recv(socketHTTPHandle_, &xml_c_[posBuff],chunkSize, 0);
        posBuff += recvLength;
        if(recvLength <= 0) // The last chunk returns 0
            break;
    }
    if (posBuff > 4)
    {
        const char *awaited_response = "HTTP/1.0 302 Found";
        if (strncmp(xml_c_, awaited_response, 18 )==0)
        {
            return true;
        }
        else
        {
            std::cerr << message_header() << "Bad response from set command. Response is :" <<  xml_c_ << std::endl;
            return false;
        }
    }
    else
    {
        std::cerr << message_header() << "Bad response from set command. Response is :" <<  xml_c_  << std::endl;
        return false;
    }
  }
}


bool FTSensor::setRDTOutputRate(unsigned int rate)
{
  if (rate > 0 && rate <= 7000)
  {
      std::stringstream cfgcomrdtrate_ss;
      cfgcomrdtrate_ss << rate;
      std::string cmd = "/comm.cgi?comrdtrate=" + cfgcomrdtrate_ss.str();

      if(sendTCPrequest(cmd))
      {
          // we consider the rate was set and don't read it back
          rdt_rate_ = rate;
          return true;
      }
      else
          return false;
  }
  else
  {
      std::cerr << message_header() << "RDT rate must be in range [1-7000]" << std::endl;
      return false;
  }
}


bool FTSensor::setGaugeBias(unsigned int gauge_idx, int gauge_bias)
{
  std::map<unsigned int, int> map;
  map[gauge_idx] = gauge_bias;
  return setGaugeBias(map);
}

std::vector<int> FTSensor::getGaugeBias()
{
  FTSensor::settings_error_t err = getSettings();
  if(err!=SETTINGS_REQUEST_ERROR && err!=GAUGE_PARSE_ERROR)
  {
     std::vector<int> bias(setbias_, setbias_ + 6);
     return bias;
  }
  else
  {
     std::cerr << message_header() << "Could not get gauge bias values" << std::endl;
     return std::vector<int>();
  }
}


bool FTSensor::setGaugeBias(std::vector<int> &gauge_vect)
{
  std::map<unsigned int, int> map;
  for (size_t i=0; i < gauge_vect.size(); ++i)
  {
    map[i] = gauge_vect[i];
  }
  return setGaugeBias(map);
}

bool FTSensor::setGaugeBias(std::map<unsigned int, int> &gauge_map)
{
  std::stringstream setbias_ss;
  std::map<unsigned int, int>::iterator it;
  bool first_element = true;
  //prepare the query
  for (it=gauge_map.begin(); it!=gauge_map.end(); ++it)
  {
    if( it->first < 6)
    {
      if(first_element)
      {
        setbias_ss << "?";
        first_element = false;
      }
      else
      {
        setbias_ss << "&";
      }
      setbias_ss << "setbias" << it->first << "=" << it->second;
    }
    else
    {
      std::cerr << message_header() << "Invalid gauge number "<< it->first << std::endl;
      return false;
    }
  }

  std::string host = getIP();
  std::string cmd = "/setting.cgi" + setbias_ss.str();
  return sendTCPrequest(cmd);
}

bool FTSensor::sendCommand()
{
  return sendCommand(cmd_.command);
}

bool FTSensor::sendCommand(uint16_t cmd)
{
  *reinterpret_cast<uint16_t*>(&request_[0]) = htons(command_s::command_header);
  *reinterpret_cast<uint16_t*>(&request_[2]) = htons(cmd);
  *reinterpret_cast<uint32_t*>(&request_[4]) = htonl(cmd_.sample_count);
  //return rt_dev_sendto(socketHandle_, (void*) &request_, sizeof(request_), 0, (sockaddr*) &addr_, addr_len_ ) == 8;
  return rt_dev_send(socketHandle_, (void*) &request_, sizeof(request_), 0) == sizeof(request_);//, (sockaddr*) &addr_, addr_len_ ) == 8;
}

bool FTSensor::getResponse()
{

  //response_ret_ = rt_dev_recvfrom(socketHandle_, (void*) &response_, sizeof(response_), 0, (sockaddr*) &addr_, &addr_len_ );
  response_ret_ = rt_dev_recv(socketHandle_, (void*) &response_, sizeof(response_), 0);//, (sockaddr*) &addr_, &addr_len_ );
  resp_.rdt_sequence = ntohl(*reinterpret_cast<uint32_t*>(&response_[0]));
  resp_.ft_sequence = ntohl(*reinterpret_cast<uint32_t*>(&response_[4]));
  resp_.status = ntohl(*reinterpret_cast<uint32_t*>(&response_[8]));
  resp_.Fx = static_cast<int32_t>(ntohl(*reinterpret_cast<int32_t*>(&response_[12 + 0 * 4])));
  resp_.Fy = static_cast<int32_t>(ntohl(*reinterpret_cast<int32_t*>(&response_[12 + 1 * 4])));
  resp_.Fz = static_cast<int32_t>(ntohl(*reinterpret_cast<int32_t*>(&response_[12 + 2 * 4])));
  resp_.Tx = static_cast<int32_t>(ntohl(*reinterpret_cast<int32_t*>(&response_[12 + 3 * 4])));
  resp_.Ty = static_cast<int32_t>(ntohl(*reinterpret_cast<int32_t*>(&response_[12 + 4 * 4])));
  resp_.Tz = static_cast<int32_t>(ntohl(*reinterpret_cast<int32_t*>(&response_[12 + 5 * 4])));
  if (response_ret_ < 0)
  {
    std::cerr << "\033[1;31m" << message_header() << "Error while receiving: " << strerror(errno) << "\033[0m" << std::endl;
  }
  if (response_ret_!=RDT_RECORD_SIZE)
    std::cerr << message_header() <<  "Error of package.xml size " <<response_ret_ << " but should be "<< RDT_RECORD_SIZE << std::endl;
  return response_ret_==RDT_RECORD_SIZE;
}

void FTSensor::doComm()
{
    if (isInitialized()) {
        if(cmd_.sample_count != 0) //do not repeat send if infinite samples
            if(!sendCommand())
                std::cerr << message_header() << "Error while sending command" << std::endl;
        if(!getResponse())
            std::cerr << message_header() << "Error while getting response, command:" <<cmd_.command << std::endl;
    }
}


void FTSensor::setBias()
{
  //std::cout << "Setting bias"<<std::endl;
  this->setSoftwareBias();
}
bool FTSensor::isInitialized()
{
    return initialized_;
}

void FTSensor::setTimeout(float sec)
{
    if (sec <= 0) {
        std::cerr << message_header() << "Can't set timeout <= 0 sec" << std::endl;
        return;
    }

    if (isInitialized()) {
        std::cerr << message_header() << "Can't set timeout if socket is initialized, call this before init()." << std::endl;
        return;
    }
  timeval_.tv_sec = static_cast<unsigned int>(sec);
  timeval_.tv_usec = static_cast<unsigned int>(sec/1.e6);
}

bool FTSensor::resetThresholdLatch()
{
  if(! sendCommand(command_s::RESET_THRESHOLD_LATCH)){
    std::cerr << message_header() << "Could not start reset threshold latch" << std::endl;
      return false;
  }
  return true;
}
bool FTSensor::setSoftwareBias()
{
  //if(!stopStreaming())
      //std::cerr << "Could not stop streaming" << std::endl;
  if(! sendCommand(command_s::SET_SOFWARE_BIAS)){
    ;//std::cerr << "Could not set software bias" << std::endl;
      return false;
  }
  //if(!startStreaming())
      //std::cerr << "Could not restart streaming" << std::endl;
  return true;
}
bool FTSensor::stopStreaming()
{
  return sendCommand(command_s::STOP);
}

bool FTSensor::startBufferedStreaming(uint32_t sample_count)
{
  setSampleCount(sample_count);
  setCommand(command_s::BUFFERED);
  if(! sendCommand()){
    std::cerr << message_header() << "Could not start buffered streaming" << std::endl;
      return false;
  }
  return true;
}
bool FTSensor::startMultiUnitStreaming(uint32_t sample_count)
{
  setSampleCount(sample_count);
  setCommand(command_s::MULTIUNIT);
  if(! sendCommand()){
    std::cerr << message_header() << "Could not start multi-unit streaming" << std::endl;
      return false;
  }
  return true;
}
bool FTSensor::startRealTimeStreaming(uint32_t sample_count)
{
  setSampleCount(sample_count);
  setCommand(command_s::REALTIME);
  if(! sendCommand()){
    std::cerr << message_header() << "Could not start realtime streaming" << std::endl;
    return false;
  }
  std::cout << message_header() << "Start realtime streaming with " << sample_count <<" samples " << std::endl;
  return true;
}

void FTSensor::setCommand(uint16_t cmd)
{
  this->cmd_.command = cmd;
}

void FTSensor::setSampleCount(uint32_t sample_count)
{
  this->cmd_.sample_count = sample_count;
}

