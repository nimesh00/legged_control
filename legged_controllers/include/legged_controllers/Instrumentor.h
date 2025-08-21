//
// Basic instrumentation profiler by Cherno

// Usage: include this header file somewhere in your code (eg. precompiled header), and then use like:
//
// Instrumentor::Get().BeginSession("Session Name");        // Begin session 
// {
//     InstrumentationTimer timer("Profiled Scope Name");   // Place code like this in scopes you'd like to include in profiling
//     // Code
// }
// Instrumentor::Get().EndSession();                        // End Session
//
// You will probably want to macro-fy this, to switch on/off easily and use things like __FUNCSIG__ for the profile name.
//
#pragma once
#include <string>
#include <chrono>
#include <algorithm>
#include <fstream>
#include <thread>
#include <mutex> // <<< ADD THIS INCLUDE

#define PROFILING 1
#if PROFILING
const char* filepath = "/home/aero/results.json";
#define PROFILE_SCOPE(name) InstrumentationTimer timer##__LINE__(name, Benchmarker_)
#else
#define PROFILE_SCOPE(name)
#endif

struct ProfileResult
{
    std::string Name;
    long long Start, End;
    uint32_t ThreadID;
};

struct InstrumentationSession
{
    std::string Name;
};

class Instrumentor
{
private:
    InstrumentationSession* m_CurrentSession;
    std::ofstream m_OutputStream;
    int m_ProfileCount;
    std::mutex m_Mutex; // <<< ADD A MUTEX MEMBER

public:
    Instrumentor()
        : m_CurrentSession(nullptr), m_ProfileCount(0)
    {
    }

    void BeginSession(const std::string& name, const std::string& filepath = "/home/aero/results.json")
    {
        std::lock_guard<std::mutex> lock(m_Mutex); // <<< ADD LOCK
        m_OutputStream.open(filepath);
        WriteHeader();
        m_CurrentSession = new InstrumentationSession{ name };
    }

    void EndSession()
    {
        std::lock_guard<std::mutex> lock(m_Mutex); // <<< ADD LOCK
        WriteFooter();
        m_OutputStream.close();
        delete m_CurrentSession;
        m_CurrentSession = nullptr;
        m_ProfileCount = 0;
    }

    void WriteProfile(const ProfileResult& result)
    {
        std::lock_guard<std::mutex> lock(m_Mutex); // <<< ADD LOCK

        if (m_ProfileCount++ > 0)
            m_OutputStream << ",";

        std::string name = result.Name;
        std::replace(name.begin(), name.end(), '"', '\'');

        m_OutputStream << "{";
        m_OutputStream << "\"cat\":\"function\",";
        m_OutputStream << "\"dur\":" << (result.End - result.Start) << ',';
        m_OutputStream << "\"name\":\"" << name << "\",";
        m_OutputStream << "\"ph\":\"X\",";
        m_OutputStream << "\"pid\":0,";
        m_OutputStream << "\"tid\":" << result.ThreadID << ",";
        m_OutputStream << "\"ts\":" << result.Start;
        m_OutputStream << "}";

        m_OutputStream.flush();
    }

    void WriteHeader()
    {
        // This function is now called from within a locked section,
        // so it doesn't need its own lock if only called by BeginSession.
        // However, adding one makes it safe for other potential uses.
        // For simplicity in this direct fix, we rely on the lock in BeginSession.
        m_OutputStream << "{\"otherData\": {},\"traceEvents\":[";
        m_OutputStream.flush();
    }

    void WriteFooter()
    {
        // Same as WriteHeader, protected by the lock in EndSession.
        m_OutputStream << "]}";
        m_OutputStream.flush();
    }

    static Instrumentor& Get()
    {
        static Instrumentor instance;
        return instance;
    }
};

// ... (InstrumentationTimer class remains unchanged) ...
class InstrumentationTimer
{
public:

    std::shared_ptr<Instrumentor> ptr;

    InstrumentationTimer(const char* name, std::shared_ptr<Instrumentor>& instrumentor_ptr) //pass by reference
        : m_Name(name), m_Stopped(false)
    {
        m_StartTimepoint = std::chrono::high_resolution_clock::now();
        ptr = instrumentor_ptr;
    }

    ~InstrumentationTimer()
    {
        if (!m_Stopped)
            Stop();
    }

    void Stop()
    {
        auto endTimepoint = std::chrono::high_resolution_clock::now();

        long long start = std::chrono::time_point_cast<std::chrono::microseconds>(m_StartTimepoint).time_since_epoch().count();
        long long end = std::chrono::time_point_cast<std::chrono::microseconds>(endTimepoint).time_since_epoch().count();

        uint32_t threadID = std::hash<std::thread::id>{}(std::this_thread::get_id());
        ptr->WriteProfile({ m_Name, start, end, threadID });

        m_Stopped = true;
    }
private:
    const char* m_Name;
    std::chrono::time_point<std::chrono::high_resolution_clock> m_StartTimepoint;
    bool m_Stopped;
};