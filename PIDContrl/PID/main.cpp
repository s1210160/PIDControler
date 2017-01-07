#include <stdio.h>
#include <fstream>

// PI����Q�C��
#define P 33.0 // ���޲�
#define I 30.0 // �ϕ��޲�
#define D 3.0
// �^�����f��
#define K 10.0 // �o�l��
#define C 11.0 // �_���s���O
#define M 1.0 // ����
#define TIME 0.1 // ���Խï��

//////////////////////////
//
// PID Controler
//
// goal:�ڕW�l, now:���ݒl
//////////////////////////

double PIDctrl(double goal, double now)
{
	static double intg = 0;		// �덷�ϕ��l
	static double err_prev = 0;	// �ꎞ���O�̌덷
	double diff;	// �덷�����l
	double err;		// �덷
	double u;		// �R���g���[���o�́i����ʁj

	// �덷
	err = goal - now;

	// �덷�����l
	diff = (err - err_prev) / TIME;

	// �덷�ϕ��l
	intg += err * TIME;

	// �������
	u = P * err + I * intg + D * diff;

	err_prev = err;

	return u;
}


void main()
{
	// �t�@�C���o��
	std::ofstream ofs("test.csv");


	double f, u;
	double acc, vel, pos, time;
	int i;

	// �����l
	vel = 0; // ���x
	pos = 0; // �ʒu
	time = 0; // ����

	for (i = 0; i < 500; i++) {
		// �R���g���[���o��
		u = PIDctrl(10, pos);

		// �O��
		f = u - (K * pos + C * vel);

		// �����x
		acc = f / M;

		// ���x
		vel += acc * TIME;

		// �ʒu
		pos += vel * TIME;

		// ����
		time += TIME;

		printf("TIME = %f, POS = %f\n", time, pos);
		ofs << time << ", " << pos << std::endl;
	}
}