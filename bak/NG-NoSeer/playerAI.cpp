#include "playerAI.h"
#include "Attack.hpp"
#include "Common.hpp"
#include "Map.hpp"
#include "TaskScheduler.hpp"
#include <cerrno>
#include <csignal>
#include <cstring>
#include <exception>

void debugTime(const std::string& name,
               Clock::time_point& ts) {
    Clock::time_point cur = Clock::now();
    debug() << name << " time = "
            << std::chrono::duration_cast<
                   std::chrono::milliseconds>(cur - ts)
                   .count()
            << " ms" << std::endl;
    ts = cur;
}
void playerAIImpl() {
    static int lastFrame = 0, skiped = 0;
    int skf = getLogic().frame - lastFrame - 1;
    skiped += skf;
    lastFrame = getLogic().frame;
    debug() << "Skiped frame = " << skiped
            << std::endl;
    debug() << "meteor = " << getLogic().meteors.size()
            << std::endl;
    debug() << "fireball = "
            << getLogic().fireballs.size()
            << std::endl;
    updateSamples(skf);
    Clock::time_point now = Clock::now();
    int mid = -1;
    std::vector<Task> trans = calcTask(mid);
    debugTime("task", now);
    std::vector<Point> nxtPos(humanSiz()), cnxt;
    std::vector<std::string> taskType(humanSiz());
    for(const Task& t : trans) {
        for(int id : t.id)
            taskType[id] = t.taskName;
        std::vector<PosInfo> res = teamMove(cnxt, t);
        for(const PosInfo& info : res)
            nxtPos[info.first] = info.second;
    }
    debugTime("move", now);
    calcAttack(mid, nxtPos, taskType);
    debugTime("attack", now);
}
void signalHandler(int sig) {
    error() << "Signal " << sig
#ifndef _WIN32
            << " " << strsignal(sig)
#endif
            << std::endl;
    throw std::runtime_error("signal");
}
template <typename T>
void setSignal(T handler) {
    signal(SIGSEGV, handler);
    // signal(SIGABRT, handler);
    signal(SIGILL, handler);
    signal(SIGFPE, handler);
// signal(SIGINT, handler);
// signal(SIGTERM, handler);
#ifndef _WIN32
    // signal(SIGQUIT, handler);
    signal(SIGPIPE, handler);
    signal(SIGSTKFLT, handler);
    signal(SIGIOT, handler);
    signal(SIGBUS, handler);
// signal(SIGKILL, handler);
#endif
}
void playerAI() {
    debug().str("");
    setSignal(signalHandler);
    errno = 0;
    try {
        playerAIImpl();
    } catch(const std::exception& exc) {
        error() << "Exception:" << exc.what()
                << std::endl;
    } catch(...) {
        error() << "Unknown error" << std::endl;
    }
    if(errno)
        error() << "Errno = " << errno << " "
                << strerror(errno) << std::endl;
    getLogic().debug(debug().str());
    setSignal(SIG_IGN);
}
