#pragma once
#include <atomic>
#include <condition_variable>
#include <deque>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
namespace planner_core {
// File I/O never runs on the controller sampling thread. Memory is bounded.
class AsyncCsvWriter {
    std::mutex mutex_;
    std::condition_variable ready_;
    std::deque<std::string> rows_;
    std::ofstream file_;
    std::thread worker_;
    std::atomic<bool> enabled_{false};
    bool stopping_=false;
    std::atomic<unsigned> dropped_{0};
public:
    ~AsyncCsvWriter(){stop();}
    void stop(){
        {std::lock_guard<std::mutex> lock(mutex_);stopping_=true;ready_.notify_one();}
        if(worker_.joinable())worker_.join();
        enabled_=false;file_.close();
    }
    void open(const std::string& path,const std::string& header){
        stop();file_.open(path);if(!file_)return;
        file_<<header;file_.flush();
        {std::lock_guard<std::mutex> lock(mutex_);stopping_=false;rows_.clear();dropped_=0;}
        enabled_=true;
        worker_=std::thread([this]{
            for(;;){
                std::string row;
                {std::unique_lock<std::mutex> lock(mutex_);ready_.wait(lock,[this]{return stopping_||!rows_.empty();});
                 if(rows_.empty()&&stopping_)break;row=std::move(rows_.front());rows_.pop_front();}
                file_<<row;
            }
            file_.flush();
        });
    }
    explicit operator bool()const{return enabled_.load();}
    void write(std::string row){
        if(!enabled_)return;
        std::lock_guard<std::mutex> lock(mutex_);
        if(stopping_)return;
        if(rows_.size()==2048){++dropped_;return;}
        rows_.push_back(std::move(row));ready_.notify_one();
    }
    unsigned dropped()const{return dropped_.load();}
};
}
