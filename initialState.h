#ifndef INITIAL_STATE_H
#define INITIAL_STATE_H

//! Модуль расчета начальных условий моделирования
#include "ICalculateElement.h"
#include "IKinematic.h"
#include "tPrimitives.hpp"

typedef struct TBaseInitialCoord_
{
    uint8_t onGround;// задать на земле, признак
    double lon;      // долгота, рад
    double lat;      // широта, рад
    double h;        // высота над уровнем моря, м
}TBaseInitialCoord;

typedef struct TOffsetInitialCoord_
{
    uint8_t onGround;// задать на земле, признак
    
    //! географические координаты опорной точки задания НУ
    double lat;      // широта, рад
    double lon;      // долгота, рад
    
    //! линейные коордианты смещения относительно опорной точки 
    double x;
    double h;        // высота над уровнем моря, м
    double z;
    //! угол поворота декартовой СК в которой производится поворот
    double a;
     
}TOffsetInitialCoord;
typedef struct TBaseInitialOwnPlane_
{
    float v_ist; //скорость истинная , м/с
    float psi;   //угол курса, рад
    float unt;   //угол наклона траектории, рад
    TBaseInitialCoord coord;
    
}TBaseInitialOwnPlane;

typedef struct TVyInitialOwnPlane_
{
    float v_ist; //скорость истинная, м/с
    float psi;   //угол курса, рад
    float v_y;   // верткальная скорость, м/с
    

    uint8_t onGround;// задать на земле, признак
    double lat;      // широта, рад
    double lon;      // долгота, рад
    double h;        // высота над уровнем моря, м   
    
}TVyInitialOwnPlane;

typedef struct TRegimeState_
{
    uint8_t start;
    uint8_t restart;
    uint8_t stop;
}TRegimeState;

typedef struct TInputButtons_
{
    uint8_t start;
    uint8_t restart;
    uint8_t stop;
}TInputButtons;

#define MAX_BUF_COORD 128
#define MAX_BUF_SPEED 128
#define MAX_BUF_ORIEN 128

enum ECoord_
{
    E_C_BASE,
    E_C_OFFSET    
};
enum ESpeed_
{
    E_S_BASE,        
};
enum EOrient_
{
    E_O_ORIENT,        
};
//! стрктура с начальными условиями
//typedef struct TInitialState_
//{
//    //! тип начальных условий 
//    uint8_t typeCoord;
//    uint8_t typeOrient;
//    uint8_t typeSpeed;
//    
//    //! данные с вариантами задания
//    uint8_t dataCoord[MAX_BUF_COORD];
//    uint8_t dataSpeed[MAX_BUF_SPEED];
//    uint8_t dataOrient[MAX_BUF_ORIEN];
//}TInitialState;

typedef struct TInitialState_
{
    TBaseInitialOwnPlane plane;
}TInitialState;
class InitialStateModel:public ICalculateElement
{
public:
    InitialStateModel(uint32_t name);
    //! обобщенный интерфейс
    virtual bool bind();
    virtual void init();
    virtual void calculate();
    virtual void finite();
private:
    void calcInitialKin();
    Trigger<uint8_t> *triggerStart;
    Trigger<uint8_t> *triggerStop;
    Trigger<uint8_t> *triggerRestart;
    //! структуры данных
    TInitialState initData;
    TRegimeState  regime;
    TInputButtons inputButton;
    TTimerIC *timer;
    //! указатель на кинематику
    IKinematicBase *pKin;
    //ICalculateElement *channel;
};

#endif
