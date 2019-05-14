#include "playerAI.h"
#include "Analyzer.hpp"
#include "Attack.hpp"
#include "Common.hpp"
#include "Map.hpp"
#include "SafeOp.hpp"
#include "TaskScheduler.hpp"
#include <cerrno>
#include <csignal>
#include <cstring>
#include <exception>

void debugTime(const std::string& name,
               Clock::time_point& ts) {
    Clock::time_point cur = Clock::now();
    info() << name << " time = "
           << std::chrono::duration_cast<
                  std::chrono::milliseconds>(cur - ts)
                  .count()
           << " ms" << std::endl;
    ts = cur;
}
void playerAIImpl() {
    static int lastFrame = 0, skiped = 0;
    SafeOp::begin();
    int skf = getLogic().frame - lastFrame - 1;
    skiped += skf;
    lastFrame = getLogic().frame;
    info() << "Skiped frame = " << skiped << std::endl;
    updateSamples(skf);
    Clock::time_point now = Clock::now(), bak = now;
    doAnalyse();
    debugTime("analyse", now);
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
    SafeOp::end();
    debugTime("frame", bak);
}
void signalHandler(int sig) {
    ERROR << "Signal " << sig
#ifndef _WIN32
          << " " << strsignal(sig)
#endif
          << std::endl;
    // throw std::runtime_error("signal");
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
    if(getLogic().frame == 1) {
        info() << "Version = " << __DATE__ << " "
               << __TIME__ << std::endl;
    }
    msg().str("");
    setSignal(signalHandler);
    errno = 0;
    try {
        playerAIImpl();
    } catch(const std::exception& exc) {
        ERROR << "Exception:" << exc.what()
              << std::endl;
    } catch(...) {
        ERROR << "Unknown error" << std::endl;
    }
    if(errno)
        ERROR << "Errno = " << errno << " "
              << strerror(errno) << std::endl;
    getLogic().debug(msg().str());
    setSignal(SIG_IGN);
}
