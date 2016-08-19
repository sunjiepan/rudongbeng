#include "can.h" 

extern   CanTxMsg TxMessage;
extern   CanRxMsg RxMessage;
extern vu16 After_filter[8];
extern vu8  After_Value[16];
extern vu8 date_out[8];
extern vu8 date_out_1[8];

extern vu8 command_level;
extern vu8 number_board ;
extern vu8 Can1_Recv_Buf[8];
vu32 mask =0;
vu16 std_id =0x000;  
vu32 ext_id =0x1314;  
/*
 * ��������CAN_GPIO_Config
 * ����  ��CAN��GPIO ����,PB8�������룬PB9�������
 * ����  ����
 * ���  : ��
 * ����  ���ڲ�����
 */
static void CAN_GPIO_Config(void)
{
 	GPIO_InitTypeDef GPIO_InitStructure;
   	
  	/*����ʱ������*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

  	/*IO����*/
	//GPIO_PinRemapConfig(GPIO_Remap1_CAN1, ENABLE);
	
	/* Configure CAN pin: RX PA11*/									          
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	             // ��������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
    
	/* Configure CAN pin: TX PA12 */									               
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		         // �����������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;    
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
}

/*
 * ��������CAN_NVIC_Config
 * ����  ��CAN��NVIC ����,��1���ȼ��飬0��0���ȼ�
 * ����  ����
 * ���  : ��
 * ����  ���ڲ�����
 */
static void CAN_NVIC_Config(void)
{
   	NVIC_InitTypeDef NVIC_InitStructure;
		/* Configure one bit for preemption priority */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	 	/*�ж�����*/
	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;	   //CAN1 RX0�ж�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;		   //��ռ���ȼ�0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;			   //�����ȼ�Ϊ0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/*
 * ��������CAN_Mode_Config
 * ����  ��CAN��ģʽ ����
 * ����  ����
 * ���  : ��
 * ����  ���ڲ�����
 */
static void CAN_Mode_Config(void)
{
   	CAN_InitTypeDef        CAN_InitStructure;
	 	/************************CANͨ�Ų�������**********************************/
	/*CAN�Ĵ�����ʼ��*/
	CAN_DeInit(CAN1);
	CAN_StructInit(&CAN_InitStructure);
	/*CAN��Ԫ��ʼ��*/
	CAN_InitStructure.CAN_TTCM=DISABLE;			   //MCR-TTCM  ʱ�䴥��ͨ��ģʽʹ��
    CAN_InitStructure.CAN_ABOM=DISABLE;			   //MCR-ABOM  �Զ����߹��� 
    CAN_InitStructure.CAN_AWUM=DISABLE;			   //MCR-AWUM  �Զ�����ģʽ
    CAN_InitStructure.CAN_NART=DISABLE;			   //MCR-NART  ��ֹ�����Զ��ش�	  DISABLE-�Զ��ش�
    CAN_InitStructure.CAN_RFLM=DISABLE;			   //MCR-RFLM  ����FIFO ����ģʽ  DISABLE-���ʱ�±��ĻḲ��ԭ�б���  
    CAN_InitStructure.CAN_TXFP=DISABLE;			   //MCR-TXFP  ����FIFO���ȼ� DISABLE-���ȼ�ȡ���ڱ��ı�ʾ�� 
    CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;  //��������ģʽ
    CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;		   //BTR-SJW ����ͬ����Ծ��� 2��ʱ�䵥Ԫ
    CAN_InitStructure.CAN_BS1=CAN_BS1_13tq;		   //BTR-TS1 ʱ���1 ռ����6��ʱ�䵥Ԫ
    CAN_InitStructure.CAN_BS2=CAN_BS2_4tq;		   //BTR-TS1 ʱ���2 ռ����3��ʱ�䵥Ԫ
    CAN_InitStructure.CAN_Prescaler =16;		   ////BTR-BRP �����ʷ�Ƶ��  ������ʱ�䵥Ԫ��ʱ�䳤�� 36/(1+13+4)/16=0.125Mbps
	CAN_Init(CAN1, &CAN_InitStructure);
}

/*
 * ��������CAN_Filter_Config
 * ����  ��CAN�Ĺ����� ����
 * ����  ����
 * ���  : ��
 * ����  ���ڲ�����
 */


  
/*���ն��ID��0x7e9,0x1800f001,ǰ��Ϊ��׼ID������Ϊ��չID��Ҫͬʱ�ܽ���������ID*/
static void CAN_Filter_Config_1(void)
{
		CAN_FilterInitTypeDef  CAN_FilterInitStructure;
		CAN_FilterInit(&CAN_FilterInitStructure);     //��ʼ��CAN_FilterInitStructrue�ṹ�����  
		CAN_FilterInitStructure.CAN_FilterNumber=0;      //���ù�������0����ΧΪ0~13  
		CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;    //���ù�������0Ϊ����ģʽ  
		CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;   //���ù�������0λ��Ϊ32λ  
			
		//��ʶλ�Ĵ���������  
		//ext_id<<3���룬����ͼ9����>>16ȡ��16λ  
		CAN_FilterInitStructure.CAN_FilterIdHigh=(((u32)ext_id<<3)&0xFFFF0000)>>16;;  //���ñ�ʶ���Ĵ������ֽڡ�  
		CAN_FilterInitStructure.CAN_FilterIdLow=(((u32)ext_id<<3)|CAN_ID_EXT|CAN_RTR_DATA)&0xFFFF; //Ҫ���˵�ID��λ 
		//����Ҳ������������  
		//CAN_FilterInitStructure.CAN_FilterIdHigh=std_id<<5;  //���ñ�ʶ���Ĵ������ֽ�.����Ϊʲô������5λ�أ�����ͼ���Կ�����CAN_FilterIdHigh��������STD[0~10]��EXID[13~17],��׼CAN ID�����ǲ�������չID���ݣ����Ϊ��Ҫ����׼CAN ID����˼Ĵ�������׼CAN ID����Ӧ����5λ����ܶ���.  
		//CAN_FilterInitStructure.CAN_FilterIdLow=0|CAN_ID_EXT;   //���ñ�ʶ���Ĵ������ֽ�,����Ҳ��������ΪCAN_ID_STD  
			
		//���μĴ���������  
		//�����˼·���Ƚ���׼CAN ID����չCAN ID��Ӧ��IDֵ������ȡ����Ϊʲô�������Ϊ���ҳ�����CAN ID����Щλ����ͬ�ģ�����ͬ��λ��˵����Ҫ���ģ���Ҫ���ĵ�λ��Ӧ��������λӦ������Ϊ1�������Ҫȡ��һ�¡��������������3λ��  
		mask =(std_id<<18);//����Ϊʲô����18λ����Ϊ��ISO11898�п��Կ�������׼CAN IDռID18~ID28��Ϊ����CAN_FilterIdHigh���룬Ӧ����2λ������Ϊ������չCAN��Ӧ����Ӧ��������16λ����ˣ��ܹ�Ӧ����2+16��18λ��Ҳ��������һ����ʽ����⣺ֱ�ӿ�Mapping�����ݣ�����STDID���EXID[0]ƫ����18λ,�������18λ.  
		mask ^=ext_id;//�������ı�׼CAN����չCAN����ȡ��  
		mask =~mask;  
		mask <<=3;//����������3λ  
		mask |=0x02; //ֻ��������֡��������Զ��֡  
		CAN_FilterInitStructure.CAN_FilterMaskIdHigh=(mask>>16)&0x0007; //�������μĴ������ֽ�  
		CAN_FilterInitStructure.CAN_FilterMaskIdLow=mask&0xffff;   //�������μĴ������ֽ�  
			
		CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0;  //�˹����������������FIFO0  
		CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //����˹�������  
		CAN_FilterInit(&CAN_FilterInitStructure); //���ù����� 
			/*CANͨ���ж�ʹ��*/
		CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
}




//static void CAN_Filter_Config(void)
//{
//   CAN_FilterInitTypeDef  CAN_FilterInitStructure;

//	/*CAN��������ʼ��*/
//	CAN_FilterInitStructure.CAN_FilterNumber=0;						//��������0
//    CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;	//�����ڱ�ʶ������λģʽ
//	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;	//������λ��Ϊ����32λ��
//	/* ʹ�ܱ��ı�ʾ�����������ձ�ʾ�������ݽ��бȶԹ��ˣ���չID�������µľ����������ǵĻ��������FIFO0�� */

//    CAN_FilterInitStructure.CAN_FilterIdHigh=(((u32)ext_id<<3)&0xFFFF0000)>>16;				//Ҫ���˵�ID��λ 
//    CAN_FilterInitStructure.CAN_FilterIdLow= (((u32)ext_id<<3)|CAN_ID_EXT|CAN_RTR_DATA)&0xFFFF; //Ҫ���˵�ID��λ 
//    CAN_FilterInitStructure.CAN_FilterMaskIdHigh= 0x0007;			//��������16λÿλ����ƥ��
//    CAN_FilterInitStructure.CAN_FilterMaskIdLow= 0xFFFF;			//��������16λÿλ����ƥ��
//	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0;//CAN_Filter_FIFO0 ;				//��������������FIFO0
//	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;			//ʹ�ܹ�����
//	CAN_FilterInit(&CAN_FilterInitStructure);
//	/*CANͨ���ж�ʹ��*/
//	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
//}


/*
 * ��������CAN_Config
 * ����  ����������CAN�Ĺ���
 * ����  ����
 * ���  : ��
 * ����  ���ⲿ����
 */
void CAN_Config(void)
{
  CAN_GPIO_Config();
  CAN_NVIC_Config();
  CAN_Mode_Config();
  //CAN_Filter_Config(); 
	CAN_Filter_Config_1();
}


/*
 * ��������CAN_SetMsg
 * ����  ��CANͨ�ű�����������
 * ����  ����
 * ���  : ��
 * ����  ���ⲿ����
 */	 
void CAN_SetMsg(void)
{	  
  //TxMessage.StdId=0x453;					 
  TxMessage.ExtId=0x1453;					 //ʹ�õ���չID
  TxMessage.IDE=CAN_ID_EXT;					 //��չģʽ
  TxMessage.RTR=CAN_RTR_DATA;				 //���͵�������
  TxMessage.DLC=8;							 //���ݳ���Ϊ8�ֽ�
	TxMessage.Data[0]=Can1_Recv_Buf[0];
	TxMessage.Data[1]=Can1_Recv_Buf[1];
	TxMessage.Data[2]=Can1_Recv_Buf[2];
  TxMessage.Data[3]=Can1_Recv_Buf[3];
	TxMessage.Data[4]=Can1_Recv_Buf[4];
  TxMessage.Data[5]=Can1_Recv_Buf[5];
	TxMessage.Data[6]=Can1_Recv_Buf[6];
  TxMessage.Data[7]=Can1_Recv_Buf[7];
//	TxMessage.Data[0]=After_Value[0];
//  TxMessage.Data[1]=After_Value[1];
//	TxMessage.Data[2]=After_Value[2];
//  TxMessage.Data[3]=After_Value[3];
//	TxMessage.Data[4]=After_Value[4];
//  TxMessage.Data[5]=After_Value[5];
//	TxMessage.Data[6]=After_Value[6];
//  TxMessage.Data[7]=After_Value[7];
}
void CAN_SetMsg_return(void)
{	  
  //TxMessage.StdId=command_level;						 
  TxMessage.ExtId=0x1314;					 //ʹ�õ���չID
  TxMessage.IDE=CAN_ID_EXT;					 //��չģʽ
  TxMessage.RTR=CAN_RTR_DATA;				 //���͵�������
  TxMessage.DLC=8;							 //���ݳ���Ϊ8�ֽ�
	TxMessage.Data[0]=date_out[0];
  TxMessage.Data[1]=date_out[1];
	TxMessage.Data[2]=date_out[2];
  TxMessage.Data[3]=date_out[3];
	TxMessage.Data[4]=date_out[4];
  TxMessage.Data[5]=date_out[5];
	TxMessage.Data[6]=date_out[6];
  TxMessage.Data[7]=date_out[7];
}

void CAN_SetMsg_1(void)
{	  
  //TxMessage.StdId=0x453;					 
  TxMessage.ExtId=0x1314;					 //ʹ�õ���չID
  TxMessage.IDE=CAN_ID_EXT;					 //��չģʽ
  TxMessage.RTR=CAN_RTR_DATA;				 //���͵�������
  TxMessage.DLC=8;							 //���ݳ���Ϊ8�ֽ�
	TxMessage.Data[0]=date_out_1[0];
  TxMessage.Data[1]=date_out_1[1];
	TxMessage.Data[2]=date_out_1[2];
  TxMessage.Data[3]=date_out_1[3];
	TxMessage.Data[4]=date_out_1[4];
  TxMessage.Data[5]=date_out_1[5];
	TxMessage.Data[6]=date_out_1[6];
  TxMessage.Data[7]=date_out_1[7];
}
/*************************END OF FILE******************************/

