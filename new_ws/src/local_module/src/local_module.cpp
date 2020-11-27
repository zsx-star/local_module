#include"local_module/pos.h"
using namespace std;

Posmodule::Posmodule():				
	m_reading_status(false),
	m_pkg_buffer(new uint8_t[500])
{
}				// 创建对象

Posmodule::~Posmodule()
{
	this->closeSerial();
	if(m_pkg_buffer!=NULL)
		delete [] m_pkg_buffer;
}              // 析构函数

bool Posmodule::openSerial(const std::string& port,int baudrate)              // 打开串口
{
	m_serial_port = new serial::Serial(port,baudrate,serial::Timeout::simpleTimeout(10)); 

	if (!m_serial_port->isOpen())
	{
		std::stringstream output;
        output << "Serial port: " << port << " failed to open." << std::endl;
		delete m_serial_port;
		m_serial_port = NULL;
		return false;
	} 
	else 
	{
		std::stringstream output;
		output << "Serial port: " << port << " opened successfully." << std::endl;
	}

	m_serial_port->flush();
	return true;
}     

void Posmodule::closeSerial()                                              // 关闭串口
{
	if(m_serial_port!=NULL)
	{
		delete m_serial_port;
		m_serial_port = NULL;
	}
}

bool Posmodule::init()  // 初始化
{
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	
	m_pub_pos = nh.advertise<nav_msgs::Odometry>("pos_topic",10);
	std::string port_name = nh_private.param<std::string>("port_name","/dev/pts/1");  // launch文件对应
	
	int baudrate = nh_private.param<int>("baudrate",115200);
	if(!openSerial(port_name,baudrate))
		return false;
	return true;
}

void Posmodule::startReading()              // 开始读取
{
	if(m_reading_status)   // 判断是否开始运行读取线程，此标志位初始时设置为false
		return ;
	m_read_thread_ptr = boost::shared_ptr<boost::thread>(new boost::thread(&Posmodule::readSerialThread,this));   // 智能指针赋值时 调用读取线程函数
}

void Posmodule::readSerialThread()
{
	m_reading_status = true;
	
	const int Max_read_size = 200;
	uint8_t * const raw_data_buf = new uint8_t[Max_read_size];
	
	size_t get_len;
		   
	while(ros::ok() && m_reading_status)    // 不停的读取数据
	{	
		try									// 异常处理
		{
			get_len = m_serial_port->read(raw_data_buf, Max_read_size);
		}
		catch(std::exception &e)
		{
			ROS_ERROR("Error reading from serial port: %s",e.what());
			ros::Duration(0.01).sleep();
			continue;
		}
		
		if(get_len == 0) 
		{
			ros::Duration(0.01).sleep();
			continue;
		}
		parseIncomingData(raw_data_buf, get_len);      // 解析数据
		
		ros::Duration(0.01).sleep();
		
		//boost::this_thread::sleep(boost::posix_time::milliseconds(10));
	}
	
	delete [] raw_data_buf;
}

void Posmodule::parseIncomingData(uint8_t* buffer,size_t len)
{
	static char pkg_buffer[63];
	static size_t pkg_buffer_index = 0; //数据包缓存定位索引
	
	size_t rCnt = 0;
	
	for(size_t i=0; i<len; ++i)
	{
		if(0 == pkg_buffer_index) 
		{
			if('m' == buffer[i])
				pkg_buffer[pkg_buffer_index++] = buffer[i];
		}
		else if(1 == pkg_buffer_index)
		{
			if('r' == buffer[i] || 'a' == buffer[i] || 'c' == buffer[i])
			{
				pkg_buffer[pkg_buffer_index++] = buffer[i];
			}
			else
				pkg_buffer_index = 0;
		}
		else
		{
			pkg_buffer[pkg_buffer_index++] = buffer[i];
			if(pkg_buffer_index == 63)
			{
				if(pkg_buffer[1] == 'a')
					parseMa(pkg_buffer);
				else if(pkg_buffer[1] == 'c')
					parseMc(pkg_buffer);
				else if (pkg_buffer[1] == 'r')
					parseMr(pkg_buffer);
										
				pkg_buffer_index = 0;
			}
		}
		
	}
}

void Posmodule::parseMa(const char* buffer)
{
	stringstream ss(buffer+3);
	double mask,TA0;
	
	ss >> hex >> mask;
	ss >> hex >> TA0;
	// 下面三位全局变量有用
	ss >> hex >> A0A1;
	ss >> hex >> A1A2;
	ss >> hex >> A0A2;
	std::cout << A0A2 << endl;
}

void Posmodule::parseMr(const char* buffer)
{
	/*
	stringstream ss(buffer+3);
	int mask,TA0,TA1,TA2,TA3;
	ss >> hex >> mask;
	ss >> hex >> TA0;
	ss >> hex >> TA1;
	ss >> hex >> TA2;
	ss >> hex >> TA3;
	*/
}

void Posmodule::parseMc(const char* buffer)
{
	stringstream ss(buffer+3);
	int mask,TA0,TA1,TA2,TA3;
	
	ss >> hex >> mask;
	ss >> hex >> TA0;
	ss >> hex >> TA1;
	ss >> hex >> TA2;
	ss >> hex >> TA3;
	std::cout << TA0 << endl;
	//三角定位
	
	double A0_x = 0,    A0_y = 0;  //  基站A0的（x，y坐标）以基站A0为坐标原点
	double A1_x = A0A1, A1_y = 0;  //  基站A1的（x，y坐标）
	double A2_x ,       A2_y;      //  基站A2的（x，y坐标）
	
	A2_x = (A0A2 * A0A2 - A1A2 * A1A2 + A0A1 * A0A1) / (2 * A0A1);
	A2_y = sqrt(A0A2 * A0A2 - A2_x * A2_x);
	
	m_pos.pose.pose.position.x = (TA0*TA0 - TA1*TA1 + A0A1*A0A1) / (2*A0A1);
	m_pos.pose.pose.position.y = (TA0*TA0 - TA2*TA2 - m_pos.pose.pose.position.x*m_pos.pose.pose.position.x + (m_pos.pose.pose.position.x- A2_x)*(m_pos.pose.pose.position.x - A2_x) + A2_y*A2_y) / (2*A2_y);
	m_pos.pose.pose.position.z = 0;
	
	m_pub_pos.publish(m_pos);
	
	std::cout << m_pos.pose.pose.position.x << endl;
	std::cout << m_pos.pose.pose.position.y << endl;
}

void Posmodule::stopReading()
{
	m_reading_status = false;
}


int main(int argc,char** argv)
{
	ros::init(argc,argv,"pos_module_node");
	
	Posmodule pos;
	
	if(pos.init())
	{
		pos.startReading();
		ROS_INFO("%s initial ok...",ros::this_node::getName().c_str());
	}
	
	ros::spin();
}



