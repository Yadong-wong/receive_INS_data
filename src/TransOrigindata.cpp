//
// Created by wyd on 2020/4/8.
//
#include "TransOrigindata.h"
class TransOriginData{  //Class "TransOriginData": receive and transform the original UDP data, and publish relative topics
private:
    ros::NodeHandle nh;
    ros::Publisher pubPoseInfoFromGps;
    ros::Publisher pubGsofOriginData;
    geometry_msgs::PoseStamped PoseInfoFromGps;
    receive_INS_data::gsof Gsof_data;   //About ROS

    int Satellite_Num;
    float HDOP;
    string Imu_alignment_status;
    string GPS_alignment_indicator;
    double Latitude;
    double Longitude;
    double Altitude;
    float North_Velocity;
    float East_Velocity;
    float Down_Velocity;
    float Total_Velocity;
    double Roll;
    double Pitch;
    double Heading;
    double Track_Angle;
    float Angular_Rate_x;
    float Angular_Rate_y;
    float Angular_Rate_z;
    float Longitudinal_acceleration;
    float Traverse_accleration;
    float Down_accleration;

    QByteArray data;
    QString buffer;
    string str;
    std::ofstream outFile;

    double GPSPosition[3];
    bool newOriginData;

public:
    TransOriginData():
    nh("~"){
        pubPoseInfoFromGps = nh.advertise<geometry_msgs::PoseStamped>("/PoseInfoFromGps",1);
        pubGsofOriginData = nh.advertise<receive_INS_data::gsof>("/OriginalGsofData",10);
        allocateMemory();

    }
    void allocateMemory()
    {
       // nh.param()
        Satellite_Num = 0;
        HDOP = 0;
        Latitude = 0;
        Longitude = 0;
        Altitude = 0;
        North_Velocity = 0;
        East_Velocity = 0;
        Down_Velocity = 0;
        Total_Velocity = 0;
        Roll = 0;
        Pitch = 0;
        Heading = 0;
        Track_Angle = 0;
        Angular_Rate_x = 0;
        Angular_Rate_y = 0;
        Angular_Rate_z = 0;
        Longitudinal_acceleration = 0;
        Traverse_accleration = 0;
        Down_accleration = 0;

        for(int i=0;i<3;i++) GPSPosition[i]=0;

        newOriginData = false;

    }

    ~TransOriginData(){}
    template <class Type >
    Type convertFromString(string str){
        long int n=0;

        Type * pf=(Type*)&n ;

        for(int i=0;i<str.length();i++)
        {
            if('0'<=str[i]&&str[i]<='9')
                str[i]=str[i]-'0';
            if('A'<=str[i]&&str[i]<='F')
                str[i]= 10+str[i]-'A';
            if('a'<=str[i]&&str[i]<='f')
                str[i]= 10+str[i]-'a';
            n=n*16+str[i];
        }
        return *pf;
    }


    void OriginDataCallback()
    {
         // str = buffer.toStdString();
          str = "0228408BC40000010A0B166D80084910BF050209103FB6CCC13F3FBCDA3F9BA4C43F55BE8C316808490B166D8003024043FE94BBFE48DC405D1646E7340AF640451ACB19BE919A3D912CBC3FB699983B985D503FB6D345BFF1082022B3F8BE3FD6C5F597715F274055C0FC255FCD4D4055C9F07DCC53D53E38A5EC3F504CCEBF0ACFDEBDD94A753DFFA7493F404E2BAB03";
       // cout << str << endl;
        if(str.length()==290)
        {
            cout.precision(16);
            string SVs_used = str.substr(30,2);
            Satellite_Num = convertFromString<int>(SVs_used);
            cout << "Satellite_Num = " << Satellite_Num << "  ";

            string hdop = str.substr(50,8);
            HDOP = convertFromString<float>(hdop);
            cout << "HDOP = " << HDOP << "  " ;

            Imu_alignment_status=str.substr(90,2);
            cout << "Imu_aligned_status = " << Imu_alignment_status << "  ";

            GPS_alignment_indicator = str.substr(92,2);
            cout << "GPS_alignment_indicator = " << GPS_alignment_indicator << "  ";
            //GPS_aligned (92,2)
            //GPS信息：经纬度、高度
            string latitude = str.substr(94,16);
            Latitude= convertFromString<double>(latitude);
            cout << "latitude = " << Latitude << "  ";

            string longitude = str.substr(110,16);
            Longitude=convertFromString<double>(longitude);
            cout << "longitude = " << Longitude << "  ";

            string altitude = str.substr(126,16);
            Altitude=convertFromString<double>(altitude);
            cout << "Altitude = " << Altitude << "  ";

            //速度信息：north、east、down三个方向速度和总速度
            string north_velocity = str.substr(142,8);
            North_Velocity=convertFromString<float>(north_velocity);

            string east_velocity = str.substr(150,8);
            East_Velocity=convertFromString<float>(east_velocity);

            string down_velocity = str.substr(158,8);
            Down_Velocity=convertFromString<float >(down_velocity);

            string total_speed = str.substr(166,8);
            Total_Velocity=convertFromString<float >(total_speed);
            cout << "Total_speed = " << Total_Velocity << "  ";

            //绕三个坐标轴的旋转角：roll、pitch 和yaw(heading)
            string roll = str.substr(174,16);
            Roll=convertFromString<double>(roll);
            cout << "Roll = " << Roll << "  ";

            string pitch =str.substr(190,16);
            Pitch=convertFromString<double>(pitch);
            cout << "Pitch = " << Pitch << "  ";

            string heading = str.substr(206,16);
            Heading = convertFromString<double>(heading);
            cout << "Heading = " << Heading << "  ";

            //Track_angle 是速度和正北方向的夹角
            string track_angle = str.substr(222,16);
            Track_Angle=convertFromString<double>(track_angle);
            cout << "Track_Angle = " << Track_Angle << "  ";

            //角速度信息
            string angular_rate_x = str.substr(238,8);
            Angular_Rate_x = convertFromString<float>(angular_rate_x);

            string angular_rate_y = str.substr(246,8);
            Angular_Rate_y=convertFromString<float >(angular_rate_y);

            string angular_rate_z = str.substr(254,8);
            Angular_Rate_z = convertFromString<float >(angular_rate_z);

            //加速度信息
            string longitudinal_acceleration = str.substr(262,8);
            Longitudinal_acceleration = convertFromString<float >(longitudinal_acceleration);

            string traverse_acceleration = str.substr(270,8);
            Traverse_accleration = convertFromString<float >(traverse_acceleration);

            string down_acceleration = str.substr(278,8);
            Down_accleration = convertFromString<float >(down_acceleration);

            //将转换的数据记录到txt文件中
            outFile.precision(16);

            outFile.open(filename.c_str(),ios::app);
            if(outFile.fail()){
                cout<<"===== Record INS_data failed =====\n";
            }
            outFile << Satellite_Num << "," << HDOP << "," << Imu_alignment_status << "," << GPS_alignment_indicator << ","
                    << Latitude << "," << Longitude << "," << Altitude << ","
                    << North_Velocity << "," << East_Velocity << "," << Down_Velocity << "," << Total_Velocity << ","
                    << Roll << "," << Pitch << "," << Heading << "," << Track_Angle << ",";

        }

    }

    void LonLat2Mercator()
    {

        double a = 6378.137;
        double e = 0.0818192;
        double k0 = 0.9996;
        double lamda;
        double lamda0;
        double v,A,T,C,S;
        double phi;

        int  E0 = 500;
        int N0 = 0;
        int zoneNumber;
        double IPI = 0.0174532925199433333333;

        zoneNumber = floor(Longitude/6)+31;
        lamda0 = ((zoneNumber-1)*6-180+3)*IPI;
        lamda = Longitude*IPI;
        phi = Latitude*IPI;
        v = 1/sqrt(1-e*e*(sin(phi)*sin(phi)));
        A = (lamda-lamda0)*cos(phi);
        T = tan(phi)*tan(phi);
        C = e*e*cos(phi)*cos(phi)/(1-e*e);
        S = (1-e*e/4-3*pow(e,4)/64-5*pow(e,6)/256)*phi -
            (3*e*e/8+3*pow(e,4)/32+45*pow(e,6)/1024)*sin(2*phi) +
            (15*pow(e,4)/256+45*pow(e,6)/1024)*sin(4*phi) -
            35*pow(e,6)/3072*sin(6*phi);
        GPSPosition[0] = (E0 + k0*a*v*(A+(1-T+C)*pow(A,3)/6 + (5-18*T+T*T)*pow(A,5)/120) - 444.4406636)*1000;
        GPSPosition[1] = (N0 + k0*a*(S+v*tan(phi)*(A*A/2+(5-T+9*C+4*C*C)*pow(A,4)/24+(61-58*T+T*T)*pow(A,6)/720)) - 4426.899512)*1000;
        GPSPosition[2] = Altitude;

        cout << "x = " << GPSPosition[0] << "  " << "y = " << GPSPosition[1] << "  " << "z = " << GPSPosition[2]   << endl;

        outFile << GPSPosition[0] << "," << GPSPosition[1] << "," << GPSPosition[2] << endl;
        outFile.close();

    }

    void handle_udp_msg(int fd)
    {
        char buf[BUFF_LEN];  //接收缓冲区，1024字节
        socklen_t len;
        int count;
        struct sockaddr_in clent_addr;  //clent_addr用于记录发送方的地址信息

        memset(buf, 0, BUFF_LEN);
        len = sizeof(clent_addr);
        //  count = read(fd,buf,BUFF_LEN);
        count = recvfrom(fd, buf, BUFF_LEN, 0, (struct sockaddr*)&clent_addr, &len);  //recvfrom是拥塞函数，没有数据就一直拥塞
        data.resize(count);
        int recv = read(fd,data.data(),count);

        if(count == -1)
        {
            printf("recieve data fail!\n");
            return;
        }
        if(recv>0)
        {
            newOriginData = true;
            buffer = byteArrayToHexStr(data);
        }
        memset(data.data(), 0, count);
        cout << count << "bytes data has been received" << endl;


    }

    QString byteArrayToHexStr(const QByteArray &data)
    {
        QString temp = "";
        QString hex = data.toHex();

        for (int i = 0; i < hex.length(); i = i + 2) {
            temp += hex.mid(i, 2);
        }

        return temp.trimmed().toUpper();
    }


    int recieve_UDP() {
        int server_fd, ret;
        struct sockaddr_in ser_addr;
        server_fd = socket(AF_INET, SOCK_DGRAM, 0); //AF_INET:IPV4;SOCK_DGRAM:UDP
        if (server_fd < 0) {
            printf("create socket fail!\n");
            return -1;
        }
        memset(&ser_addr, 0, sizeof(ser_addr));
        ser_addr.sin_family = AF_INET;
        ser_addr.sin_addr.s_addr = htonl(INADDR_ANY);  //IP地址，需要进行网络序转换，INADDR_ANY：本地地址
        ser_addr.sin_port = htons(SERVER_PORT);     //端口号，需要网络序转换
        //  ret = connect(server_fd,(struct sockaddr *) &ser_addr, sizeof(ser_addr));
        ret = bind(server_fd, (struct sockaddr *) &ser_addr, sizeof(ser_addr));
        if (ret < 0) {
            printf("socket bind fail!\n");
            return -1;
        }
        handle_udp_msg(server_fd);   //处理接收到的数据
        close(server_fd);
        return 0;
    }

    void PublishData(){

        Gsof_data.header.stamp = ros::Time::now();
        Gsof_data.header.frame_id = "/base_link";
        Gsof_data.full_data = str;
        Gsof_data.Satellite_Num = Satellite_Num;
        Gsof_data.DOP_info = HDOP;
        Gsof_data.Imu_aligned = Imu_alignment_status;
        Gsof_data.latitude = Latitude;
        Gsof_data.longitude = Longitude;
        Gsof_data.altitude = Altitude;
        Gsof_data.north_v = North_Velocity;
        Gsof_data.east_v = East_Velocity;
        Gsof_data.down_v = Down_Velocity;
        Gsof_data.total_v = Total_Velocity;
        Gsof_data.roll = Roll;
        Gsof_data.pitch = Pitch;
        Gsof_data.heading = Heading;
        Gsof_data.track_angle = Track_Angle;
        Gsof_data.angle_rate_x = Angular_Rate_x;
        Gsof_data.angle_rate_y = Angular_Rate_y;
        Gsof_data.angle_rate_z = Angular_Rate_z;
        Gsof_data.longitudinal_a = Longitudinal_acceleration;
        Gsof_data.traverse_a = Traverse_accleration;
        Gsof_data.down_a = Down_accleration;

        pubGsofOriginData.publish(Gsof_data);

        PoseInfoFromGps.header.stamp = ros::Time::now();
        PoseInfoFromGps.header.frame_id = "/base_link";
        PoseInfoFromGps.pose.position.x = GPSPosition[0];
        PoseInfoFromGps.pose.position.y = GPSPosition[1];
        PoseInfoFromGps.pose.position.z = GPSPosition[2];

        geometry_msgs::Quaternion getQuat = tf::createQuaternionMsgFromRollPitchYaw(Roll,Pitch,Heading);
        PoseInfoFromGps.pose.orientation.x = getQuat.x;
        PoseInfoFromGps.pose.orientation.y = getQuat.y;
        PoseInfoFromGps.pose.orientation.z = getQuat.z;
        PoseInfoFromGps.pose.orientation.w = getQuat.w;
        pubPoseInfoFromGps.publish(PoseInfoFromGps);

    }

    void run()
    {

        if(recieve_UDP()==0 && newOriginData == true){  //如果UDP连接成功并且新数据的标志位是“true”，则进行组合导航原始数据的处理
            newOriginData=false;

            OriginDataCallback(); //解析GSOF的数据

            LonLat2Mercator();  //将经纬度转换成UTM坐标系，单位是m

            PublishData();  //发布原始的的GPS数据以及AGV的三维位姿

        }

    }

};


int main(int argc, char* argv[]) {

    ros::init(argc, argv, "TransInsData");
    //定义文件名称:"filename + time"
    time_t currentTime=time(NULL);//注意NULL大小写
    char chCurrentTime[256];
    strftime(chCurrentTime,sizeof(chCurrentTime),"%Y%m%d %H%M%S",localtime(&currentTime));
    stCurrentTime = chCurrentTime;
    filename = "INS_data"+stCurrentTime+".txt";

    TransOriginData TD;

    ROS_INFO("\033[47;33m---->\\033[0m transform start.\"");

    ros::Rate rate(100);
    while(ros::ok())
    {
        ros::spinOnce();

        TD.run();

        rate.sleep();
    }
    return 0;


}