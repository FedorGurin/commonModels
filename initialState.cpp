#include "initialState.h"


#include "IMsg.h"
#include "globalNameID.h"
#include "math_func.h"
#include "hal.h"
InitialStateModel::InitialStateModel(uint32_t name):ICalculateElement(name)
{
    //! обнуление внутренних параметров
    memset((void*)&initData,    0, sizeof(initData));
    memset((void*)&regime,      0, sizeof(regime));
    memset((void*)&inputButton, 0, sizeof(inputButton));

    pKin    = 0;
    //channel = 0;
   
    timer = new TTimerIC();
    createTimer(timer,2500);
    //! по умолчанию переходим в режим "Паузы"
    regime.restart = 1;
    inputButton.stop = 0;
    
    
    triggerRestart  = new Trigger<uint8_t>(0);
    triggerStop     = new Trigger<uint8_t>(0);
    triggerStart    = new Trigger<uint8_t>(0);
    
    //! начальные условия по умолчанию
    initData.plane.coord.lon      = gradToRadian(38.127312777777);
    initData.plane.coord.lat      = gradToRadian(55.5641416666666);
    initData.plane.v_ist          = 0.0001;
    initData.plane.coord.h        = 1000.0;
    initData.plane.coord.onGround = 1;
    initData.plane.psi            = 0 * gradToRad;
    initData.plane.unt            = 0.0;

    //! добавление зависимостей
    bindTo(new TEventAddrStat(ID_I_KinematicObj,   (uintptr_t)&pKin,   TEventAddrStat::GET_POINTER,this));
    //bindTo(new TEventAddrStat(ID_CHANNEL_SYS,     (uintptr_t)&channel,   TEventAddrStat::GET_POINTER,this));
    share(ID_STATE_START,   (uintptr_t)&regime.start);
    share(ID_STATE_STOP,    (uintptr_t)&regime.stop);
    share(ID_STATE_RESTART, (uintptr_t)&regime.restart);

    ///////////////////////////////////////
    setFreq(ICalculateElement::Hz6_25);
    setStart();
}

//! обязательный интерфейс
bool InitialStateModel::bind()
{
    return ICalculateElement::bind();
}

void InitialStateModel::init()
{
    //! регистрация параметров
    TEventAddrStat eventAddr(ID_MPPM,
                             0,
                             TEventAddrStat::GET_POINTER,
                              this,
                              TEventAddrStat::E_UP,
                              0);
    eventAddr.addr = 0;
    MsgRegMPPM msgRegMppm(&eventAddr); //! сообщение
    msgRegMppm.addReq(ptrToReq("InitialState_Init",  &initData));
    msgRegMppm.addReq(ptrToReq("Reg_Cur",            &regime));
    msgRegMppm.addReq(ptrToReq("InputButton_Input",  &inputButton));

    ICalculateElement::eEvent(&msgRegMppm);

    ICalculateElement::init();
    
    regime.restart = 1;
    inputButton.stop = 0;
    timer->restart();
    timer->start();    
}

void InitialStateModel::calculate()
{
    triggerRestart->setState(inputButton.restart);
    triggerStart->setState(inputButton.start);
    triggerStop->setState(inputButton.stop);

    if(triggerStop->isHighFront() == true && regime.restart == 0)
    {
        pKin->setStop();
        //channel->setStop();
        regime.stop  = 1;
        regime.start = 0;
    }

    if(triggerStart->isHighFront() == true && regime.restart == 0)
    {
        regime.start   = 1;
        regime.stop    = 0;
        regime.restart = 0;
        pKin->setStart();
        //channel->setStart();
    }

    if(triggerRestart->isHighFront() == true && regime.start == 0)
    {
        regime.start   = 0;
        regime.stop    = 0;
        regime.restart = 1;
        timer->restart();
        timer->start();
        //channel->setStart();
    }
    
    if(regime.restart == 1)
        calcInitialKin();
    
    if(timer->isLimit() == true )
    {
        timer->stop();
        timer->restart();
        regime.restart = 0;
        regime.stop    = 1;
        pKin->setStop();
        //channel->setStop();
    }
}
void InitialStateModel::calcInitialKin()
{
	//pKin->kin.clear();
	pKin->kin.n_c.y = g_acc;
	pKin->kin.n_c.x = 0.0;
	pKin->kin.n_c.z = 0.0;

    pKin->kin.tan       = 0; //3 * gradToRad;
    pKin->kin.gamma     = 0; //TEST: 3 * gradToRad;

    pKin->kin.Vc    = initData.plane.v_ist;
    if(initData.plane.coord.onGround)
    {
        pKin->kin.c_g.y = pKin->kin.dHRelief + 2.0;
        pKin->kin.swAirOn = 0;
    }
    else
    {
        pKin->kin.c_g.y = initData.plane.coord.h;
        pKin->kin.swAirOn = 1;
   }
    pKin->kin.psi   = kursToPsi(initData.plane.psi);
    pKin->kin.unt   = initData.plane.unt;

    pKin->kin.lam_geo   = pKin->kin.lam0_geo  = initData.plane.coord.lon;
    pKin->kin.fi_geo    = pKin->kin.fi0_geo   = initData.plane.coord.lat;

    pKin->initialData();
}
void InitialStateModel::finite()
{

}
