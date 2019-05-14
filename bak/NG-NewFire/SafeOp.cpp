#include "SafeOp.hpp"
namespace SafeOp {
    static FT sumMove = 0.0;
    static int fireCnt = 0, meteorCnt = 0,
               frameCnt = 0;
    void move(int num, const Point& p,
              const std::string& callPos) {
        if(num >= humanSiz()) {
            error() << "move:invaild num" << std::endl;
            return;
        }
        Point cp = getMyUnit(num).position;
        debug() << "move " << num << " from [" << cp.x
                << " " << cp.y << "] to [" << p.x
                << " " << p.y
                << "] dist=" << dist(cp, p)
                << std::endl;
        if(MapHelper::isWall(p) ||
           dist(cp, p) > CONST::flash_distance) {
            error() << "move:invalid pos from "
                    << callPos << std::endl;
            return;
        }
        sumMove += dist(cp, p);
        getLogic().move(num, p);
    }
    void fire(int num, const Point& p) {
        if(num >= humanSiz()) {
            error() << "fire:invaild num" << std::endl;
            return;
        }
        if(!canFire(getMyUnit(num))) {
            error() << "fire:can not fire"
                    << std::endl;
            return;
        }
        ++fireCnt;
        debug() << "shoot " << num << " at [" << p.x
                << " " << p.y << "]" << std::endl;
        getLogic().shoot(num, p);
    }
    void flash(int num) {
        if(num >= humanSiz()) {
            error() << "flash:invaild num"
                    << std::endl;
            return;
        }
        if(!canFlash(getMyUnit(num))) {
            error() << "flash:can not flash"
                    << std::endl;
            return;
        }
        debug() << "flash " << num << std::endl;
        getLogic().flash(num);
    }
    void meteor(int num, const Point& p) {
        if(num >= humanSiz()) {
            error() << "meteor:invaild num"
                    << std::endl;
            return;
        }
        if(!canMeteor(getMyUnit(num))) {
            error() << "meteor:can not meteor"
                    << std::endl;
            return;
        }
        debug() << "meteor " << num << " in [" << p.x
                << " " << p.y << "]" << std::endl;
        ++meteorCnt;
        getLogic().meteor(num, p);
    }
    void profile() {
        FT frame = getLogic().frame;
        for(int i = 0; i < humanSiz(); ++i)
            frameCnt += getMyUnit(i).hp > 0;
        debug() << "alive ratio:"
                << 100.0 * frameCnt /
                (frame * humanSiz())
                << "%" << std::endl;
        FT totFlash =
            frameCnt / CONST::human_flash_interval;
        FT totMove = (frameCnt - totFlash) *
                CONST::human_velocity +
            totFlash * CONST::flash_distance;
        debug() << "move ratio:"
                << 100.0 * sumMove / totMove << "%"
                << std::endl;
        FT totFire =
            frameCnt / CONST::human_fireball_interval;
        debug() << "fire ratio:"
                << 100.0 * fireCnt / totFire << "%"
                << std::endl;
        FT totMeteor =
            frameCnt / CONST::human_meteor_interval;
        debug() << "meteor ratio:"
                << 100.0 * meteorCnt / totMeteor << "%"
                << std::endl;
    }
}
