/***************************************************************************************
                                      声 明
    本项目代码仅供个人学习使用，可以自由移植修改，但必须保留此声明信息。移植过程中出现
其他不可估量的BUG，天际智联不负任何责任。请勿商用！

程序版本：1.0
程序日期：2018-1-26
程序作者：愤怒的小孩 E-mail：1138550496@qq.com
版权所有：西安天际智联信息技术有限公司
****************************************************************************************/
#include "filter.h"
#include "structconfig.h"
#include "math.h"

#define N 20      //滤波缓存数组大小
#define M_PI_F 3.1416f

/*******************************************************************************
*函  数 ：float FindPos(float*a,int low,int high)
*功  能 ：确定一个元素位序
*参  数 ：a  数组首地址
*         low数组最小下标
*         high数组最大下标
*返回值 ：返回元素的位序low
*备  注 : 无
*******************************************************************************/
float FindPos(float*a,int low,int high)
{
    float val = a[low];                      //选定一个要确定值val确定位置
    while(low<high)
    {
        while(low<high && a[high]>=val)
             high--;                       //如果右边的数大于VAL下标往前移
             a[low] = a[high];             //当右边的值小于VAL则复值给A[low]

        while(low<high && a[low]<=val)
             low++;                        //如果左边的数小于VAL下标往后移
             a[high] = a[low];             //当左边的值大于VAL则复值给右边a[high]
    }
    a[low] = val;//
    return low;
}

/*******************************************************************************
*函  数 ：void QuiteSort(float* a,int low,int high)
*功  能 ：快速排序
*参  数 ：a  数组首地址
*         low数组最小下标
*         high数组最大下标
*返回值 ：无
*备  注 : 无
*******************************************************************************/
 void QuiteSort(float* a,int low,int high)
 {
     int pos;
     if(low<high)
     {
         pos = FindPos(a,low,high); //排序一个位置
         QuiteSort(a,low,pos-1);    //递归调用
         QuiteSort(a,pos+1,high);
     }
 }

 /*******************************************************************************
*函  数 ：float  SortAver_Filter(float value)
*功  能 ：去最值平均值滤波一组数据
*参  数 ：value 采样的数据
*返回值 ：返回滤波后的数据
 *备  注 : 无
*******************************************************************************/
void  SortAver_Filter(float value,float *filter,uint8_t n)
{
	static float buf[N] = {0.0};
	static uint8_t cnt =0,flag = 1;
	float temp=0;
	uint8_t i=0;
	buf[cnt++] = value;
	if(cnt<n && flag) 
		return;  //数组填不满不计算	
	else flag=0; 
  QuiteSort(buf,0,n-1);
	for(i=1;i<n-1;i++)
	 {
		temp += buf[i];
	 }

	 if(cnt>=n) cnt = 0;

	 *filter = temp/(n-2);
}

 /*******************************************************************************
*函  数 ：float  SortAver_Filter1(float value)
*功  能 ：去最值平均值滤波一组数据
*参  数 ：value 采样的数据
*返回值 ：返回滤波后的数据
 *备  注 : 无
*******************************************************************************/
void  SortAver_Filter1(float value,float *filter,uint8_t n)
{
	static float buf[N];
	static uint8_t cnt =0 ,flag = 1;
	float temp=0;
	uint8_t i;
	buf[cnt++] = value;
	if(cnt<n && flag) 
		return;   //数组填不满不计算
	else
		flag = 0;
  QuiteSort(buf,0,n-1);
	for(i=1;i<n-1;i++)
	 {
		temp += buf[i];
	 }
	if(cnt>=n) cnt = 0;
	 
	*filter = temp/(n-2);

}

/*******************************************************************************
*函  数 ：void  SortAver_FilterXYZ(INT16_XYZ *acc,FLOAT_XYZ *Acc_filt,uint8_t n)
*功  能 ：去最值平均值滤波三组数据
*参  数 ：*acc 要滤波数据地址
*         *Acc_filt 滤波后数据地址
*返回值 ：返回滤波后的数据
*备  注 : 无
*******************************************************************************/
void  SortAver_FilterXYZ(INT16_XYZ *acc,FLOAT_XYZ *Acc_filt,uint8_t n)
{
	static float bufx[N],bufy[N],bufz[N];
	static uint8_t cnt =0,flag = 1;
	float temp1=0,temp2=0,temp3=0;
	uint8_t i;
	bufx[cnt] = acc->X;
	bufy[cnt] = acc->Y;
	bufz[cnt] = acc->Z;
	cnt++;      //这个的位置必须在赋值语句后，否则bufx[0]不会被赋值
	if(cnt<n && flag) 
		return;   //数组填不满不计算
	else
		flag = 0;
	
  QuiteSort(bufx,0,n-1);
	QuiteSort(bufy,0,n-1);
	QuiteSort(bufz,0,n-1);
	for(i=1;i<n-1;i++)
	 {
		temp1 += bufx[i];
		temp2 += bufy[i];
		temp3 += bufz[i];
	 }

	 if(cnt>=n) cnt = 0;
	 Acc_filt->X  = temp1/(n-2);
	 Acc_filt->Y  = temp2/(n-2);
	 Acc_filt->Z  = temp3/(n-2);
}

/*******************************************************************************
*函  数 ：void Aver_FilterXYZ6(INT16_XYZ *acc,INT16_XYZ *gry,FLOAT_XYZ *Acc_filt,
                              FLOAT_XYZ *Gry_filt,uint8_t n)
*功  能 ：滑动窗口滤波六组数据
*参  数 ：*acc ,*gry 要滤波数据地址
*         *Acc_filt,*Gry_filt 滤波后数据地址
*返回值 ：返回滤波后的数据
*备  注 : 无
*******************************************************************************/
void Aver_FilterXYZ6(INT16_XYZ *acc,INT16_XYZ *gry,FLOAT_XYZ *Acc_filt,FLOAT_XYZ *Gry_filt,uint8_t n)
{
  static float bufax[N],bufay[N],bufaz[N],bufgx[N],bufgy[N],bufgz[N];
	static uint8_t cnt =0,flag = 1;
	float temp1=0,temp2=0,temp3=0,temp4=0,temp5=0,temp6=0;
	uint8_t i;
	bufax[cnt] = acc->X;
	bufay[cnt] = acc->Y;
	bufaz[cnt] = acc->Z;
	bufgx[cnt] = gry->X;
	bufgy[cnt] = gry->Y;
	bufgz[cnt] = gry->Z;
	cnt++;      //这个的位置必须在赋值语句后，否则bufax[0]不会被赋值
	if(cnt<n && flag) 
		return;   //数组填不满不计算
	else
		flag = 0;
	for(i=0;i<n;i++)
	{
		temp1 += bufax[i];
		temp2 += bufay[i];
		temp3 += bufaz[i];
		
		temp4 += bufgx[i];
		temp5 += bufgy[i];
		temp6 += bufgz[i];
	}
	 if(cnt>=n) cnt = 0;
	 Acc_filt->X  = temp1/n;
	 Acc_filt->Y  = temp2/n;
	 Acc_filt->Z  = temp3/n;
	
	 Gry_filt->X  = temp4/n;
	 Gry_filt->Y  = temp5/n;
	 Gry_filt->Z  = temp6/n;
	
}

/*******************************************************************************
*函  数 ：void Aver_FilterXYZ(INT16_XYZ *acc,FLOAT_XYZ *Acc_filt,uint8_t n)
*功  能 ：滑动窗口滤波三组数据
*参  数 ：*acc  要滤波数据地址
*         *Acc_filt 滤波后数据地址
*返回值 ：返回滤波后的数据
*备  注 : 无
*******************************************************************************/
void Aver_FilterXYZ(INT16_XYZ *acc,FLOAT_XYZ *Acc_filt,uint8_t n)
{
  static int32_t bufax[N],bufay[N],bufaz[N];
	static uint8_t cnt =0,flag = 1;
	int32_t temp1=0,temp2=0,temp3=0,i;
	bufax[cnt] = acc->X;
	bufay[cnt] = acc->Y;
	bufaz[cnt] = acc->Z;
	cnt++;      //这个的位置必须在赋值语句后，否则bufax[0]不会被赋值
	if(cnt<n && flag) 
		return;   //数组填不满不计算
	else
		flag = 0;
	for(i=0;i<n;i++)
	{
		temp1 += bufax[i];
		temp2 += bufay[i];
		temp3 += bufaz[i];
	}
	 if(cnt>=n)  cnt = 0;
	 Acc_filt->X  = temp1/n;
	 Acc_filt->Y  = temp2/n;
	 Acc_filt->Z  = temp3/n;
}

/*******************************************************************************
*函  数 ：void Aver_Filter(float data,float *filt_data,uint8_t n
*功  能 ：滑动窗口滤波一组数据
*参  数 ：data  要滤波数据
*         *filt_data 滤波后数据地址
*返回值 ：返回滤波后的数据
*备  注 : 无
*******************************************************************************/
void Aver_Filter(float data,float *filt_data,uint8_t n)
{
  static float buf[N];
	static uint8_t cnt =0,flag = 1;
	float temp=0;
	uint8_t i;
	buf[cnt] = data;
	cnt++;
	if(cnt<n && flag) 
		return;   //数组填不满不计算
	else
		flag = 0;
	
	for(i=0;i<n;i++)
	{
		temp += buf[i];
	}
	if(cnt>=n) cnt = 0;
	 *filt_data = temp/n;
}

/*******************************************************************************
*函  数 ：void Aver_Filter1(float data,float *filt_data,uint8_t n
*功  能 ：滑动窗口滤波一组数据
*参  数 ：data  要滤波数据
*         *filt_data 滤波后数据地址
*返回值 ：返回滤波后的数据
*备  注 : 无
*******************************************************************************/
void Aver_Filter1(float data,float *filt_data,uint8_t n)
{
  static float buf[N];
	static uint8_t cnt =0,flag = 1;
	float temp=0;
	uint8_t i;
	buf[cnt++] = data;
	if(cnt<n && flag) 
		return;   //数组填不满不计算
	else
		flag = 0;
	for(i=0;i<n;i++)
	{
		temp += buf[i];
	}
	if(cnt>=n) cnt = 0;
	 *filt_data = temp/n;
}


typedef struct
{
	float lpf_1;

	float out;
}_lf_t;

#define ABS(x) ( (x)>0?(x):-(x) )
#define LIMIT( x,min,max ) ( (x) < (min)  ? (min) : ( (x) > (max) ? (max) : (x) ) )

void LPF_1(float hz,float time,float in,float *out)  
{
	*out += ( 1 / ( 1 + 1 / ( hz *6.28f *time ) ) ) *( in - *out );

}

void limit_filter(float T,float hz,_lf_t *data,float in)
{
	float abs_t;
	LPF_1(hz,T,	 in,&(data->lpf_1)); 
	abs_t = ABS(data->lpf_1);
	data->out = LIMIT(in,-abs_t,abs_t);

}

//#define FILTER_NUM	5   //滤波平均值的除数
//#define FILTER_A	3.0f  //滤波限幅大小

///*限幅度平均值滤波*/
//void presssureFilter(float* in, float* out)
//{	
//	static u8 i=0;
//	static float filter_buf[FILTER_NUM]={0.0};
//	double filter_sum=0.0;
//	u8 cnt=0;	
//	float deta;

//	if(filter_buf[i] == 0.0f)
//	{
//		filter_buf[i]=*in;
//		*out=*in;
//		if(++i>=FILTER_NUM)	i=0;
//	} else 
//	{
//		if(i) deta=*in-filter_buf[i-1];
//		else deta=*in-filter_buf[FILTER_NUM-1];
//		
//		if(fabs(deta)<FILTER_A)
//		{
//			filter_buf[i]=*in;
//			if(++i>=FILTER_NUM)	i=0;
//		}
//		for(cnt=0;cnt<FILTER_NUM;cnt++)
//		{
//			filter_sum+=filter_buf[cnt];
//		}
//		*out=filter_sum /FILTER_NUM;
//	}
//}

/* Private functions ---------------------------------------------------------
float LowPassFilter_1st(float newData, float Fcut, float deltaT, LowPassFilter_1st_TypeDef *TypeDef)
{  
	TypeDef->lpf_factor =  deltaT / (deltaT + 1 / (2 * PI * Fcut)); 
	TypeDef->data_out = TypeDef->oldData * (1 - TypeDef->lpf_factor) + newData * TypeDef->lpf_factor;
	TypeDef->oldData = TypeDef->data_out;
	return TypeDef->data_out;
}

float LowPassFilter_2nd(float newData, float Fcut, float deltaT, LowPassFilter_2nd_TypeDef *TypeDef) 
{  
	TypeDef->a = 1 / (2 * PI * Fcut * deltaT);    
	TypeDef->b0 = 1 / (TypeDef->a*TypeDef->a + 2*TypeDef->a + 1);  
	TypeDef->a1 = (2*TypeDef->a*TypeDef->a + 2*TypeDef->a) / (TypeDef->a*TypeDef->a + 2*TypeDef->a + 1);  
	TypeDef->a2 = (TypeDef->a*TypeDef->a) / (TypeDef->a*TypeDef->a + 2*TypeDef->a + 1);
	TypeDef->data_out = newData * TypeDef->b0 + TypeDef->lastout * TypeDef->a1 - TypeDef->preout * TypeDef->a2;  
	TypeDef->preout = TypeDef->lastout;  
	TypeDef->lastout = TypeDef->data_out;  
	return TypeDef->data_out; 
}*/
