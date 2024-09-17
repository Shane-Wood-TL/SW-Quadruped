//------------------------------------------------------------------------------------------------
//Extra Math
float pytherm(float sidea, float sideb); //returns hypotenuse c
float raddec(float rad); //radians to degress
float loc(float a, float b, float c); // law of cosines, returns angle c in degress 
float pythermhypt(float sidea, float sidec); //returns side b
float decrad(float deg); //degrees to radians


//------------------------------------------------------------------------------------------------
//Modes
void standing_0();
void IK_1(float xAngleV, float yAngleV, float zAngleV);
void FWalk_2(float yAngleV, float zAngleV);
void FTurn_3(float yAngleV, float zAngleV);
void User_4(float yAngleV, float zAngleV);

void Default_9();
void wakeup_9();


//------------------------------------------------------------------------------------------------
void getData();



void applyBaseOffsets();