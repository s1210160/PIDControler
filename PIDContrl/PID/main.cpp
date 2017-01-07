#include <stdio.h>
#include <fstream>

// PI制御ゲイン
#define P 33.0 // 比例ｹﾞｲﾝ
#define I 30.0 // 積分ｹﾞｲﾝ
#define D 3.0
// 運動モデル
#define K 10.0 // バネ力
#define C 11.0 // ダンピング
#define M 1.0 // 質量
#define TIME 0.1 // 時間ｽﾃｯﾌﾟ

//////////////////////////
//
// PID Controler
//
// goal:目標値, now:現在値
//////////////////////////

double PIDctrl(double goal, double now)
{
	static double intg = 0;		// 誤差積分値
	static double err_prev = 0;	// 一時刻前の誤差
	double diff;	// 誤差微分値
	double err;		// 誤差
	double u;		// コントローラ出力（制御量）

	// 誤差
	err = goal - now;

	// 誤差微分値
	diff = (err - err_prev) / TIME;

	// 誤差積分値
	intg += err * TIME;

	// 制御入力
	u = P * err + I * intg + D * diff;

	err_prev = err;

	return u;
}


void main()
{
	// ファイル出力
	std::ofstream ofs("test.csv");


	double f, u;
	double acc, vel, pos, time;
	int i;

	// 初期値
	vel = 0; // 速度
	pos = 0; // 位置
	time = 0; // 時刻

	for (i = 0; i < 500; i++) {
		// コントローラ出力
		u = PIDctrl(10, pos);

		// 外力
		f = u - (K * pos + C * vel);

		// 加速度
		acc = f / M;

		// 速度
		vel += acc * TIME;

		// 位置
		pos += vel * TIME;

		// 時刻
		time += TIME;

		printf("TIME = %f, POS = %f\n", time, pos);
		ofs << time << ", " << pos << std::endl;
	}
}