#define _CRT_SECURE_NO_WARNINGS

#include <stdlib.h>
#include <time.h>
#include "glut.h"
#include "main.h"
#include "GlobalVariable.h"
#include "Inline.h"
#include "GLFunction.h"
#include "Display.h"
#include "file.h"

int RorI; /*RANSACとIAのどちらを用いるか(RANSAC or IA)*/
int Vehicle_Type;	/*車種 0:自転車　1:バイク　2:microEV*/

int g_numdata ;

DWORD start;                /* プログラムの実行時間計測 start */
DWORD end;                    /* プログラムの実行時間計測 end */
TrackingObject tracking;


/*/////////////////////////////////////////////////////////////////////
メイン関数
/////////////////////////////////////////////////////////////////////*/
int main(int argc, char *argv[]){
    
    srand((unsigned int)time(NULL));
	
    const int START_DATA_READ = 0;        /* データリードで読み飛ばす時間 */

    const int WPX = 0;        /* 描画ウィンドウ表示位置 x */
    const int WPY = 0;        /* 描画ウィンドウ表示位置 y */

    int count = 0;            /* データリードを開始する時間までループを回すカウント */
    
    g_numdata = DataNumSelect();

    /*路面検出をRANSACかIAのどちらでおこなうか　0:RANSAC, 1:IA */
	std::cout << "RANSAC or IA? 0:RANSAC, 1:IA" << std::endl;
	std::cin >> RorI;


	std::cout << "Vehicle Type Bicycle:0, Bike:1,car:2" << std::endl;
	std::cin >> Vehicle_Type;

    /* ファイルオープン */
    MicroevFileopen();

    /* ファイル読み飛ばし */
    while (count <START_DATA_READ){
        DataSkip();
        count++;
    }

    char windowName[NAME_ARREY_SIZE] = { 0 };            /* ウィンドウの名前 */

    /* ウィンドウの名前を設定 */
    sprintf_s(windowName, NAME_ARREY_SIZE, "g_numdata %d", g_numdata);

	/* グリッドマップ全初期化 */
	for (int i = 0; i < GRIDMAP_SIZE + GRIDMAP_SIZE_MARGIN * 2; i++) {
		for (int j = 0; j < GRIDMAP_SIZE + GRIDMAP_SIZE_MARGIN * 2; j++) {
			/* グリッドマップ原点からの各セルの距離を計算 */
			g_gridmap[i][j].distance = sqrt((double)((i - 133)*(i - 133))
				+ (double)((j -133)*(j - 133))) * CELL_SIZE;
			/* グリッドマップ原点に対して各セルの位置する方角を計算 */
			g_gridmap[i][j].theta = calcDegree(0.0, 0.0, (double)(i - 133), (double)(j - 133));
		}
	}

    /* OpenGL初期化 */
    glutInitWindowPosition(WPX, WPY);
    glutInitWindowSize(WIDTH, HEIGHT);
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
    glutCreateWindow(windowName);
    Init();
    glutIdleFunc(0);
    glutKeyboardFunc(Keyboard);
    glutSpecialFunc(SpecialKey);
    glutDisplayFunc(Display);
    glutMotionFunc(Motion);
    glutMouseFunc(Mouse);
    glutReshapeFunc(Resize);
    glutMainLoop();

    /* ファイルクローズ */
    MicroevFileclose();

    return 0;
}


/*/////////////////////////////////////////////////////////////////////
実験データ選択関数
引数：なし
返り値：実験のデータ番号
/////////////////////////////////////////////////////////////////////*/
int DataNumSelect(void) {
	const int DATA_SIZE = 44;
	int num;
	int datanum;
	int data[DATA_SIZE];
	int i;

	data[0] = 14031;
	data[1] = 338947;
	data[2] = 338604;
	data[3] = 339393;
	data[4] = 937739;
	data[5] = 938089;
	data[6] = 938422;
	data[7] = 938729;
	data[8] = 939087;
	data[9] = 931077;
	data[10] = 3011307;
	data[11] = 2098;//往路600scan
	data[12] = 2421;//往路500scan
	data[13] = 2913;//往路650scan
	data[14] = 3395;//往路600scan
	data[15] = 3845;//往路600scan
	data[16] = 4147;//往路600scan
	data[17] = 4499;//往路778scan
	data[18] = 4818;//往路500scan
	data[19] = 5122;//往路600scan
	data[20] = 5696;//往路500scan
	data[21] = 6001;//往路450scan
	data[22] = 6379;//往路450scan
	data[23] = 6772;//往路550scan
	data[24] = 8492;//○
	data[25] = 8706;//○
	data[26] = 8930;//○
	data[27] = 9114;//○
	data[28] = 9338;//○
	data[29] = 9616;//○
	data[30] = 9765;//往路600scan
	data[31] = 10073;//往路600scan
	data[32] = 10295;//往路600scan
	data[33] = 10560;//往路600scan
	data[34] = 1736115;//四輪9
	data[35] = 1735503;//四輪7
	data[36] = 1734374;//四輪4
	data[37] = 1734817;//四輪5
	data[38] = 1735127;//四輪6往路700scan
	data[39] = 604021;//手押し1
	data[40] = 1740756;//20car
	data[41] = 1741058;//四輪10073
	data[42] = 433415 ;//5048scan
	data[43] = 3008949;


	std::cout << "datanum?" << std::endl;
	for (i = 0; i < DATA_SIZE; i++) {
		std::cout << std::setw(2) << i << " : " << data[i] << std::endl;
	}

	std::cin >> num;

	datanum = data[num];

	return datanum;
}

/*/////////////////////////////////////////////////////////////////////
2次元平面上の2点A(ax,ay)とB(bx,by)のなす角度を算出
引数：なし
返り値：なし
/////////////////////////////////////////////////////////////////////*/
double calcDegree(double ax, double ay, double bx, double by){
    double dir;
    double tx, ty;    /***  tanθ = ty/tx  ***/

    tx = bx - ax;
    ty = by - ay;

    /* アークタンジェントで角度算出 */
    dir = RadianToDigree(atan(ty / tx));
    //printf("arctan = %lf\n", dir);

    /* 第１象限or第３象限 */
    if (dir >= 0){
        if (tx >= 0 && ty >= 0);     /* 第１象限 → そのまま */
        else          dir += 180;     /* 第３象限 → + 180 */
    }
    /* 第２象限or第４象限 */
    else{
        if (tx < 0 && ty >= 0) dir += 180;    /* 第２象限 → + 180 */
        else                   dir += 360;    /* 第４象限 → + 360 */
    }
    //printf("方位角は%lfです．\n", dir);
    return dir;
}
