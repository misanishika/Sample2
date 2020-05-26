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

int RorI; /*RANSAC��IA�̂ǂ����p���邩(RANSAC or IA)*/
int Vehicle_Type;	/*�Ԏ� 0:���]�ԁ@1:�o�C�N�@2:microEV*/

int g_numdata ;

DWORD start;                /* �v���O�����̎��s���Ԍv�� start */
DWORD end;                    /* �v���O�����̎��s���Ԍv�� end */
TrackingObject tracking;


/*/////////////////////////////////////////////////////////////////////
���C���֐�
/////////////////////////////////////////////////////////////////////*/
int main(int argc, char *argv[]){
    
    srand((unsigned int)time(NULL));
	
    const int START_DATA_READ = 0;        /* �f�[�^���[�h�œǂݔ�΂����� */

    const int WPX = 0;        /* �`��E�B���h�E�\���ʒu x */
    const int WPY = 0;        /* �`��E�B���h�E�\���ʒu y */

    int count = 0;            /* �f�[�^���[�h���J�n���鎞�Ԃ܂Ń��[�v���񂷃J�E���g */
    
    g_numdata = DataNumSelect();

    /*�H�ʌ��o��RANSAC��IA�̂ǂ���ł����Ȃ����@0:RANSAC, 1:IA */
	std::cout << "RANSAC or IA? 0:RANSAC, 1:IA" << std::endl;
	std::cin >> RorI;


	std::cout << "Vehicle Type Bicycle:0, Bike:1,car:2" << std::endl;
	std::cin >> Vehicle_Type;

    /* �t�@�C���I�[�v�� */
    MicroevFileopen();

    /* �t�@�C���ǂݔ�΂� */
    while (count <START_DATA_READ){
        DataSkip();
        count++;
    }

    char windowName[NAME_ARREY_SIZE] = { 0 };            /* �E�B���h�E�̖��O */

    /* �E�B���h�E�̖��O��ݒ� */
    sprintf_s(windowName, NAME_ARREY_SIZE, "g_numdata %d", g_numdata);

	/* �O���b�h�}�b�v�S������ */
	for (int i = 0; i < GRIDMAP_SIZE + GRIDMAP_SIZE_MARGIN * 2; i++) {
		for (int j = 0; j < GRIDMAP_SIZE + GRIDMAP_SIZE_MARGIN * 2; j++) {
			/* �O���b�h�}�b�v���_����̊e�Z���̋������v�Z */
			g_gridmap[i][j].distance = sqrt((double)((i - 133)*(i - 133))
				+ (double)((j -133)*(j - 133))) * CELL_SIZE;
			/* �O���b�h�}�b�v���_�ɑ΂��Ċe�Z���̈ʒu������p���v�Z */
			g_gridmap[i][j].theta = calcDegree(0.0, 0.0, (double)(i - 133), (double)(j - 133));
		}
	}

    /* OpenGL������ */
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

    /* �t�@�C���N���[�Y */
    MicroevFileclose();

    return 0;
}


/*/////////////////////////////////////////////////////////////////////
�����f�[�^�I���֐�
�����F�Ȃ�
�Ԃ�l�F�����̃f�[�^�ԍ�
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
	data[11] = 2098;//���H600scan
	data[12] = 2421;//���H500scan
	data[13] = 2913;//���H650scan
	data[14] = 3395;//���H600scan
	data[15] = 3845;//���H600scan
	data[16] = 4147;//���H600scan
	data[17] = 4499;//���H778scan
	data[18] = 4818;//���H500scan
	data[19] = 5122;//���H600scan
	data[20] = 5696;//���H500scan
	data[21] = 6001;//���H450scan
	data[22] = 6379;//���H450scan
	data[23] = 6772;//���H550scan
	data[24] = 8492;//��
	data[25] = 8706;//��
	data[26] = 8930;//��
	data[27] = 9114;//��
	data[28] = 9338;//��
	data[29] = 9616;//��
	data[30] = 9765;//���H600scan
	data[31] = 10073;//���H600scan
	data[32] = 10295;//���H600scan
	data[33] = 10560;//���H600scan
	data[34] = 1736115;//�l��9
	data[35] = 1735503;//�l��7
	data[36] = 1734374;//�l��4
	data[37] = 1734817;//�l��5
	data[38] = 1735127;//�l��6���H700scan
	data[39] = 604021;//�艟��1
	data[40] = 1740756;//20car
	data[41] = 1741058;//�l��10073
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
2�������ʏ��2�_A(ax,ay)��B(bx,by)�̂Ȃ��p�x���Z�o
�����F�Ȃ�
�Ԃ�l�F�Ȃ�
/////////////////////////////////////////////////////////////////////*/
double calcDegree(double ax, double ay, double bx, double by){
    double dir;
    double tx, ty;    /***  tan�� = ty/tx  ***/

    tx = bx - ax;
    ty = by - ay;

    /* �A�[�N�^���W�F���g�Ŋp�x�Z�o */
    dir = RadianToDigree(atan(ty / tx));
    //printf("arctan = %lf\n", dir);

    /* ��P�ی�or��R�ی� */
    if (dir >= 0){
        if (tx >= 0 && ty >= 0);     /* ��P�ی� �� ���̂܂� */
        else          dir += 180;     /* ��R�ی� �� + 180 */
    }
    /* ��Q�ی�or��S�ی� */
    else{
        if (tx < 0 && ty >= 0) dir += 180;    /* ��Q�ی� �� + 180 */
        else                   dir += 360;    /* ��S�ی� �� + 360 */
    }
    //printf("���ʊp��%lf�ł��D\n", dir);
    return dir;
}
