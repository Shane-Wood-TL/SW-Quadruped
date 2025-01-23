#ifndef CordsClass
#define CordsClass
class Cords{
  public:
    float xH;
    float xFB;
    float xLR;
    float xRot;
    float yRot;
    float zRot;
    Cords(){
        xH = 0;
        xFB = 0;
        xLR = 0;
        xRot = 0;
        yRot = 0;
        zRot = 0;
    }
    void updateCords(float xHv, float xFBv, float xLRv, float xRotv, float yRotv, float zRotv){
        xH = xHv;
        xFB = xFBv;
        xLR = xLRv;
        xRot = xRotv;
        yRot = yRotv;
        zRot = zRotv;
    }
    void updatePosition(float xHv, float xFBv, float xLRv){
        xH = xHv;
        xFB = xFBv;
        xLR = xLRv;
    }
};
#endif