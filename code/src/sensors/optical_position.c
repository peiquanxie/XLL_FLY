#include "optical_position.h"
#include "stabilizer_task.h"
#include "pmw3901.h"
#include "maths.h"

#define X 0
#define Y 1

#define RESOLUTION			(0.2131946f)/*1m�߶��� 1�����ض�Ӧ��λ�ƣ���λcm*/
#define OULIER_LIMIT 		(100)		/*������������޷�*/
#define VEL_LIMIT			(150.f)		/*�����ٶ��޷�*/

opFlow_t opFlow;	/*����*/
motionBurst_t currentMotion;
static u8 outlierCount = 0;			/*���ݲ����ü���*/



/*��λ��������*/
static void resetOpFlowData(void)
{
	for(u8 i=0; i<2; i++)
	{
		opFlow.pixSum[i] = 0;
		opFlow.pixComp[i] = 0;
		opFlow.pixValid[i] = 0;
		opFlow.pixValidLast[i] = 0;
	}
}


void Get_OpticalData(void)
{	
	static u16 count = 0;
	opFlow.isOpFlowOk = true;
		
	readMotion(&currentMotion);
	
	 if(currentMotion.minRawData == 0 && currentMotion.maxRawData == 0)
	 {
			if(count++ > 100 && opFlow.isOpFlowOk == true)
			{
				count = 0;
				opFlow.isOpFlowOk = false;		/*��������*/
			}		
		}else
		{
			count = 0;
		}
		
	 if(opFlow.isOpFlowOk ==true)
	 {
		/*����2֮֡������ر仯������ʵ�ʰ�װ������� (pitch:x)  (roll:y)*/
		int16_t pixelDx = currentMotion.deltaX;
		int16_t pixelDy = -currentMotion.deltaY;

		if (ABS(pixelDx) < OULIER_LIMIT && ABS(pixelDy) < OULIER_LIMIT) 
		{
			opFlow.pixSum[X] += pixelDx;  //�ƶ����ۻ�����
			opFlow.pixSum[Y] += pixelDy;
		}else 
		{
			outlierCount++;
		}
	}
}

#ifdef AVERAGE_FILTER

#define GROUP		2
#define FILTER_NUM	3

/*�޷���ֵ�˲���*/
void velFilter(float* in, float* out)
{	
	static u8 i=0;
	static float filter_buf[GROUP][FILTER_NUM] = {0.0};
	double filter_sum[GROUP] = {0.0};		
	
	for(u8 j=0;j<GROUP;j++)
	{
		if(filter_buf[j][i] == 0.0f)
		{
			filter_buf[j][i] = in[j];
			out[j] = in[j];			
		} else 
		{			
			if(fabs(in[j]) < VEL_LIMIT)
			{
				filter_buf[j][i] = in[j];
			}
			for(u8 cnt=0;cnt<FILTER_NUM;cnt++)
			{
				filter_sum[j] += filter_buf[j][cnt];
			}
			out[j]=filter_sum[j] /FILTER_NUM;
		}
	}
	if(++i >= FILTER_NUM)	i = 0;
}
#endif

bool Optical_Position_Update(state_t *state, float dt)
{
	static u8 cnt = 0;
	float height = 0.01f * state->position.z;/*��ȡ�߶���Ϣ ��λm*/
	
	Get_OpticalData();
	
	if(opFlow.isOpFlowOk && height<4.0f)	/*4m��Χ�ڣ���������*/
	{
		cnt= 0;
		opFlow.isDataValid = true;
		
		float coeff = RESOLUTION * height;
		float tanRoll = tanf(state->attitude.roll * DEG2RAD);
		float tanPitch = tanf(state->attitude.pitch * DEG2RAD);
		
//		opFlow.pixComp[X] = 480.f * tanPitch;	/*���ز�����������*/
//		opFlow.pixComp[Y] = 480.f * tanRoll;
		opFlow.pixComp[X] = 0;	/*���ز�����������*/
		opFlow.pixComp[Y] = 0;
		opFlow.pixValid[X] = (opFlow.pixSum[X] + opFlow.pixComp[X]);	/*ʵ���������*/
		opFlow.pixValid[Y] = (opFlow.pixSum[Y] + opFlow.pixComp[Y]);		
		
		if(height < 0.05f)	/*����������Χ����5cm*/
		{
			coeff = 0.0f;
		}
		opFlow.deltaPos[X] = coeff * (opFlow.pixValid[X] - opFlow.pixValidLast[X]);	/*2֮֡��λ�Ʊ仯������λcm*/
		opFlow.deltaPos[Y] = coeff * (opFlow.pixValid[Y] - opFlow.pixValidLast[Y]);	
		opFlow.pixValidLast[X] = opFlow.pixValid[X];	/*��һ��ʵ���������*/
		opFlow.pixValidLast[Y] = opFlow.pixValid[Y];
		opFlow.deltaVel[X] = opFlow.deltaPos[X] / dt;	/*�ٶ� cm/s*/
		opFlow.deltaVel[Y] = opFlow.deltaPos[Y] / dt;
		
		
#ifdef AVERAGE_FILTER
		velFilter(opFlow.deltaVel, opFlow.velLpf);		/*�޷���ֵ�˲���*/
#else
		opFlow.velLpf[X] += (opFlow.deltaVel[X] - opFlow.velLpf[X]) * 0.15f;	/*�ٶȵ�ͨ cm/s*/
		opFlow.velLpf[Y] += (opFlow.deltaVel[Y] - opFlow.velLpf[Y]) * 0.15f;	/*�ٶȵ�ͨ cm/s*/
#endif			
		opFlow.velLpf[X] = constrainf(opFlow.velLpf[X], -VEL_LIMIT, VEL_LIMIT);	/*�ٶ��޷� cm/s*/
		opFlow.velLpf[Y] = constrainf(opFlow.velLpf[Y], -VEL_LIMIT, VEL_LIMIT);	/*�ٶ��޷� cm/s*/
		state->velocity.x = opFlow.velLpf[X];
		state->velocity.y = opFlow.velLpf[Y];
	
		opFlow.posSum[X] += opFlow.deltaPos[X];	/*�ۻ�λ�� cm*/
		opFlow.posSum[Y] += opFlow.deltaPos[Y];	/*�ۻ�λ�� cm*/
		state->position.x = opFlow.posSum[X];
		state->position.y = opFlow.posSum[Y];
	}
	else if(opFlow.isDataValid == true)
	{
		if(cnt++ > 100)	/*��������߶ȣ��л�Ϊ����ģʽ*/
		{
			cnt = 0;
			opFlow.isDataValid = false;
		}	
		resetOpFlowData();	
	}
	
	return opFlow.isOpFlowOk;	/*���ع���״̬*/
}


/*��ȡ��������״̬*/
bool getOpDataState(void)
{
	return opFlow.isDataValid;
}


void reset_Position(void)
{
	resetOpFlowData();
	opFlow.posSum[X]=0;
	opFlow.posSum[Y]=0;
	state.velocity.x =0;
	state.velocity.y =0;
	state.position.x = 0;
	state.position.y = 0;
}
