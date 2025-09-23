
#include <iostream>

// 定义一个用于打印的辅助类
class GlobalPrinter {
public:
    // 构造函数中实现打印
    GlobalPrinter(const char* msg) {
        std::cout << "全局区域打印：" << msg << std::endl;
    }
};

// 定义全局变量，触发构造函数执行（在main前打印）
GlobalPrinter printer1("程序初始化开始");
GlobalPrinter printer2("正在加载配置...");

#include <FastLioProcess.h>


int main(int argc, char** argv){
    // if(true){
    //     // 将打印信息保存到txt文件里
    //     std::ofstream out("output.txt");
    //     // 保存原始cout缓冲区，以便后续恢复
    //     std::streambuf* cout_buf = std::cout.rdbuf();
    //     // 将cout重定向到文件
    //     std::cout.rdbuf(out.rdbuf());
    // }
    
    cout << "开始00" <<endl;
    std::cout << std::fixed << std::setprecision(15);
    cout << "开始01" <<endl;
    ros::init(argc, argv, "laserMapping"); // 注册节点
    cout << "开始02" <<endl;
    ros::NodeHandle nh; // 创建节点权柄 读取launch的参数和发布、订阅
    cout << "开始0" <<endl;
    // 有个类初始化接收节点权柄，进行接收launch中的参数，存有很多属性（相对全局，状态管理类，数据管理类，地图管理类），用于调用，
    // 该类为主类，包含点云处理，迭代
    // 1. 创建初始化对象 (包含读取参数、设置发布器和接收器（回调）)
    FastLioProcess process(nh);
    cout << "开始1" <<endl;
    // 2. 开始运行主体程序
    process.run(); 
    return 0;
}