
#include "macrodriver.h"
#include "cgc.h"
#include "userdefine.h"


void R_CGC_Create(void)   
{
   CMC = _00_CGC_HISYS_PORT | _00_CGC_SYSOSC_UNDER10M; //P121,P122Ϊ�˿�ģʽ,���ⲿ��������

//����fmx:ʱ������״̬���ƼĴ���CSC.MSTOP ����ϵͳʱ�ӵ����п���λ
   MSTOP = 1U;     //X1�񵴵�·ֹͣ

//����fmain:ϵͳʱ�ӿ��ƼĴ���CKC.MCM0
   MCM0 = 0U;     //ѡ������ڲ�����ʱ��(fIH=24M����Ϊ��ϵͳʱ��(fMAIN��
    
//��ѡ������ڲ�������Ϊ12λ�����ʱ��������ʱ��;��ѡ������ڲ�������Ϊ��ʱ�� RJ �ļ���Դ
   OSMC = _10_CGC_IT_TMRJ_CLK_FIL;

//����fIH:ʱ������״̬���ƼĴ���CSC.HIOSTOP �����ڲ���������
   HIOSTOP = 0U;   //�����ڲ���������
}













