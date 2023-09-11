#ifndef __DEBUG_MANAGER__
#define __DEBUG_MANAGER__

#include <string>
#include <map>
#include <vector>
#include <ros/ros.h>
#include <fstream>
#include <iostream>

class DebugManager {

public:

    

private:
    
    std::unordered_map<std::string, ros::Publisher> m_vPublisherList;

};

// 计时类，用于统计算法运行时间
class TimeDebugger {

public: 
    void NewLine() {
		gettimeofday(&start, NULL);
        current_line_time = 0;
        record_list.push_back(std::map<std::string, double>());
    }
    double DebugTime(const std::string& algo_name) {
        struct timeval end;
        gettimeofday(&end, NULL);
		double dtime = (end.tv_sec - start.tv_sec) * 1000.0 +(end.tv_usec - start.tv_usec) * 0.001;
        record_list.back()[algo_name] += dtime;
        current_line_time += dtime;
        gettimeofday(&start, NULL);
        return dtime;
    }
    double GetCurrentLineTime() {
        record_list.back()["all"] = current_line_time;
        return current_line_time;
    }
    void CoutCurrentLine() {
        for(auto && [name,time] : record_list.back()) {
            std::cout << name << ": " << time << " | ";
        }
        std::cout << "\n";
    }
    void OutputDebug(const std::string& file_name) {
        std::ofstream file(file_name);
        for(auto && [name,_] : record_list.back())
            file << "," << name;
        file << std::endl;
        for(auto && record : record_list) {
            for(auto && [_,time] : record)
                file << "," << time;
            file << std::endl;
        }
        file.close();
    }

private:
    struct timeval start;
    double current_line_time;
    std::vector<std::map<std::string, double>> record_list;
};


#include <mutex>
#include <memory>

// 线程安全的计时类
class TimeDebuggerThread {

    friend class TimeDebuggerProxy;

public:
    
    void OutputDebug(const std::string& file_name) {
        std::unique_lock<std::mutex> lock(record_mutex);
        std::ofstream file(file_name);
        for(auto && [name,_] : record_list.back())
            file << "," << name;
        file << std::endl;
        for(auto && record : record_list) {
            for(auto && [_,time] : record)
                file << "," << time;
            file << std::endl;
        }
        file.close();
    }

private:

    void UpdateLine(const size_t id, const std::map<std::string, double>& record) {
        std::unique_lock<std::mutex> lock(record_mutex);
        while(record_list.size() <= id) 
            record_list.push_back(std::map<std::string,double>());
        record_list[id] = record;
    }

    std::mutex record_mutex;
    std::vector<std::map<std::string, double>> record_list;
};

// 记录单行数据的代理，由于线程安全的计时类不可以直接被使用，因此，需要代理类进行接管
class TimeDebuggerProxy {

public:

    TimeDebuggerProxy(size_t id):id(id), current_line_time(0) {
        gettimeofday(&start, NULL);
    }
    TimeDebuggerProxy(size_t id, TimeDebuggerThread* debugger_ptr):id(id), debugger(debugger_ptr),current_line_time(0){
        gettimeofday(&start, NULL);
    }

    ~TimeDebuggerProxy() {
        if(debugger != nullptr)
            debugger->UpdateLine(id, record);
    }

    double DebugTime(const std::string& algo_name) {
        struct timeval end;
        gettimeofday(&end, NULL);
        double dtime = (end.tv_sec - start.tv_sec) * 1000.0 +(end.tv_usec - start.tv_usec) * 0.001;
        record[algo_name] = dtime;
        current_line_time += dtime;
        gettimeofday(&start, NULL);
        return dtime;
    }    

    double GetCurrentLineTime() {
        record["all"] = current_line_time;
        return current_line_time;
    }

private:
    std::map<std::string, double> record;
    struct timeval start;
    double current_line_time;
    TimeDebuggerThread* debugger;
    size_t id;
};



#endif