#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <set>
#include <map>
#include <cmath>

using namespace std;

/**
*** On each turn the pods movements are computed this way:
***   - Target: if the angle to the target exceeds 18 degrees, it is bounded (except for the 1st round).
***   - Thrust: the normalized vector between the position of the pod and its target is multiplied by the given thrust value. The result is added to the current speed vector.
***   - Movement: The speed vector is added to the position of the pod. If a collision would occur at this point, the pods rebound off each other.
***   - Friction: the current speed vector of each pod is multiplied by 0.85
***   - The speed's values are truncated and the position's values are rounded to the nearest integer.
*** Collisions are elastic. The minimum impulse of a collision is 120.
*** A boost is in fact an acceleration of 650.
*** A shield multiplies the Pod mass by 10.
*** The provided angle is absolute. 0° means facing EAST while 90° means facing SOUTH.
**/

namespace Config
{
    static float const FORCE_SHIELD_RADIUS = 400;
    static float const CHECKPOINT_RADIUS = 600;
    static float const ANGULAR_SPEED = 18;
    static float const POD_RADIUS = 400;
}

template<typename T>
class Vector2D
{
public:
    typedef T data_type;
    typedef Vector2D<data_type> self_type;

    Vector2D() : x(0), y(0) {}
    Vector2D(data_type x, data_type y) : x(x), y(y) {}
    data_type x;
    data_type y;

    self_type Normalize() const
    {
        return self_type(x, y) / Length();
    }

    float Length() const
    {
        return std::sqrt(x*x + y*y);
    }

    self_type operator-(self_type const & other) const
    {
        return self_type(x - other.x, y - other.y);
    }

    self_type operator+(self_type const & other) const
    {
        return self_type(x + other.x, y + other.y);
    }

    self_type operator/(data_type n) const
    {
        return self_type(x / n, y / n);
    }

    self_type operator*(data_type n) const
    {
        return self_type(x * n, y * n);
    }

    float Dot(self_type const & other) const
    {
        return x * other.x + y * other.y;
    }

    template<typename type>
    Vector2D<type> Convert() const
    {
        return Vector2D<type>(static_cast<type>(x), static_cast<type>(y));
    }
};

typedef Vector2D<float> Point;
typedef Vector2D<float> Vector;
typedef Vector2D<float> Position;
typedef Vector2D<int>   Pointi;


class CheckPoint
{
    public:
        CheckPoint() : _position(0, 0) {}
        CheckPoint(Position const & pos) : _position(pos) {}
        CheckPoint(float x, float y) : _position(x, y) {}

        Position const & Pos() const { return _position; }
    private:
        Position _position;
};

class Layout
{
    public:
        Layout()
        : _currentCP(0)
        , _nbTurn(0)
        , _boostIdx(0)
        {

        }
        bool InsertCPIFN(Position const & pos)
        {
            if (_knownCP.find(std::make_pair(pos.x, pos.y)) == _knownCP.end())
            {
                std::cerr << "Insert Checkpoint" << std::endl;
                _checkpoints.push_back(CheckPoint(pos));
                _knownCP.insert(std::make_pair(pos.x, pos.y));
                return true;
            }
            return false;
        }
        CheckPoint const & GetCurrentCP()
        {
            return GetCheckPoint(_currentCP);
        }
        CheckPoint const & GetNextCheckPoint()
        {
            return GetCheckPoint(_currentCP + 1);
        }
        CheckPoint const & GetCheckPoint(int nb)
        {
            return _checkpoints[nb % _checkpoints.size()];
        }

        unsigned int CheckPointIdx() const
        {
            return _currentCP;
        }
        void NextCP()
        {
            _currentCP += 1;
            if (_currentCP >= _checkpoints.size())
            {
                _currentCP == 0;
                ++_nbTurn;
            }
        }

        unsigned int Turn() const
        {
            return _nbTurn;
        }

        void SetCurrentCP(Position const & pos)
        {
            unsigned int lastCPIdx = _currentCP;
            for (unsigned int i = 0; i < _checkpoints.size(); ++i)
            {
                if (_checkpoints[i].Pos().x == pos.x && _checkpoints[i].Pos().y == pos.y)
                {
                    _currentCP = i;
                }
            }
            if (lastCPIdx > _currentCP)
            {
                ++_nbTurn;
                _ComputeMaxDistBetweenCheckPoint();
            }
        }

        std::vector<CheckPoint> const & CheckPoints() const
        {
            return _checkpoints;
        }

        unsigned int IsLongerDistBetweenCP() const
        {
            return _currentCP == _boostIdx; // buggait
        }
    private:
        void _ComputeMaxDistBetweenCheckPoint()
        {
            float maxDistSq = 0;
            for (unsigned int i = 0; i < _checkpoints.size(); ++i)
            {
                CheckPoint const & currentCP = GetCheckPoint(i);
                CheckPoint const & nextCP = GetCheckPoint(i+1);
                float dist = std::pow(currentCP.Pos().x - nextCP.Pos().x, 2.0f) + std::pow(currentCP.Pos().y - nextCP.Pos().y, 2.0f);
                if (dist > maxDistSq)
                {
                    maxDistSq = dist;
                    _boostIdx = (i + 1) % _checkpoints.size();
                }
            }
        }
        unsigned int            _boostIdx;
        unsigned int            _nbTurn;
        unsigned int            _currentCP;
        std::vector<CheckPoint> _checkpoints;
        std::set<std::pair<int, int>> _knownCP;
};

void SendDirection(Pointi const & target, int speed, bool useBoost)
{
    if (!useBoost)
        cout << target.x << " " << target.y << " " << speed << endl;
    else
        cout << target.x << " " << target.y << " " << "BOOST" << endl;
}

std::istream & operator>>(std::istream & stream, Position & pos)
{
    stream >> pos.x >> pos.y;
    return stream;
}

std::ostream & operator<<(std::ostream & stream, Position const & pos)
{
    stream << "(" << pos.x << ", " << pos.y << ")";
    return stream;
}

class WorldState
{
public:
    WorldState() {}
    Position const & PodPos()           const { return _podPos; }
    Position const & EnemyPos()         const { return _enemyPos; }
    Position const & CheckPointPos()    const { return _checkPointPos; }
    float            CheckPointAngles() const { return _checkPointAngles; }
    float            CheckPointDist()   const { return _checkPointDist; }

    void Init()
    {
        cin >> _podPos
            >> _checkPointPos
            >> _checkPointDist
            >> _checkPointAngles;
            cin.ignore();
        cin >> _enemyPos; cin.ignore();
    }
private:
    Position _podPos;
    Position _enemyPos;
    Position _checkPointPos;
    float    _checkPointAngles;
    float    _checkPointDist;
};

class World
{
public:
    World() { }
    void Update()
    {
        _previousState = _state;
        _state.Init();
    }

    WorldState const & State() const { return _state; }
    WorldState const & PreviousState() const { return _state; }
private:
    WorldState _state;
    WorldState _previousState;
};

float Degree2Rad(float angle)
{
    return angle / M_PI * 180.0f;
}

float AngleBetween(Vector const & v1, Vector const & v2)
{
    return std::acos(v1.Normalize().Dot(v2.Normalize()));
}

class Pod
{
public:
    class PodState
    {
    public:
        void Update()
        {
            std::cin >> _pos >> _speed >> _angleInDegree >> _nextCheckPointId;
        }

        Position const & Pos() const { return _pos; }
        Vector const & Speed() const { return _speed; }
        float Angle() const { return _angleInDegree; }
        int NextCP() const { return _nextCheckPointId; }
    private:
        Position _pos;
        Vector   _speed;
        float    _angleInDegree;
        int      _nextCheckPointId;
    };

    struct Command
    {
        bool UseBoost = false;
        bool UseShield = false;
        Position Target;
        int Speed = 0;

        void Reset()
        {
            UseBoost = false;
            UseShield = false;
            Target = Position(0, 0);
            Speed = 0;
        }
    };

    Pod() {}

    void Update()
    {
        _currentState.Update();
    }

    void SetCommand(Command const & cmd)
    {
        _command = cmd;
    }

    void Commit()
    {
        Pointi target = _command.Target.Convert<int>();
        std::cout << target.x << " " << target.y << " ";
        if (_command.UseBoost) std::cout << "BOOST";
        else if (_command.UseShield) std::cout << "SHIELD";
        else std::cout << _command.Speed;
        std::cout << endl;
        _command.Reset();
    }

    PodState const & CurrentState() const { return _currentState; }
private:
    PodState _currentState;
    Command  _command;
};

int main()
{
    Layout layout;
    float previousDist = 0;
    Position previousPos;
    int nbLaps, nbCheckPoint;
    cin >> nbLaps >> nbCheckPoint; cin.ignore();
    for (int i = 0; i < nbCheckPoint; ++i)
    {
        Position checkpointPos;
        cin >> checkpointPos; cin.ignore();
        layout.InsertCPIFN(checkpointPos);
    }
    Pod myPods[2];
    Pod enemyPods[2];
    while (1)
    {
        myPods[0].Update();
        myPods[1].Update();
        enemyPods[0].Update();
        enemyPods[1].Update();
        {
            for (int i = 0; i < 2; ++i)
            {
                Pod::Command command;

                command.Speed = 100;
                command.Target = layout.GetCheckPoint(myPods[i].CurrentState().NextCP()).Pos();

                myPods[i].SetCommand(command);
            }
        }
        myPods[0].Commit();
        myPods[1].Commit();

        /*if (diffDist > 0)
        {
            nextCheckpointAngle = Degree2Rad(Angle(podPos - previousPos, checkpointPos - podPos));
        }

        bool usePrediction = layout.Turn() > 0;
        layout.SetCurrentCP(checkpointPos);




        Point target = checkpointPos;
        // mesure anti orbite
        if (nextCheckpointDist < Config::CHECKPOINT_RADIUS * 4)
        {
            Vector toTargetDir = (checkpointPos - podPos).Normalize();
            Vector normalToTarget(toTargetDir.y, - toTargetDir.x);

            speed = 75;
            if (layout.Turn() > 0)
            {
                Point target1 = checkpointPos + normalToTarget * Config::CHECKPOINT_RADIUS * 0.60;
                Point target2 = checkpointPos - normalToTarget * Config::CHECKPOINT_RADIUS * 0.60;
                if ((target2 - layout.GetNextCheckPoint().Pos()).Length() > (target1 - layout.GetNextCheckPoint().Pos()).Length())
                {
                    target = target1;
                }
                else
                {
                    target = target2;
                }
            }
        }

        if (diffDist < 0 && nextCheckpointDist < Config::CHECKPOINT_RADIUS * 2)
            speed = 0;
        if (nextCheckpointAngle > 70 || nextCheckpointAngle < -70)
            speed = 20;

        previousDist = nextCheckpointDist;
        previousPos = podPos;
*/
        //SendDirection(target.Convert<int>(), speed, layout.Turn() == 1 && layout.IsLongerDistBetweenCP() && nextCheckpointAngle < 20 && nextCheckpointAngle > -20);
    }
}
