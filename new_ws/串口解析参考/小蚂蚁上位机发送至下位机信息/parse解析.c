//�����ϲ���������͵Ŀ�������
void USART1_IRQHandler(void)                	          //����1�жϷ������  �����ж�һ�ξͽ�һ�δ����жϺ���
{
	unsigned char rCh;                                    // char��һ���ֽڣ�����ÿ�δӴ��ڶ�ȡ���Ǹ��ֽڣ�
	static char rCnt = 0;							      // ��λ����
	static u8 RecBuf[recBufLen]={0x00};
	uint8_t RecCheckSum=0;
	u8 index;
	unsigned char rCh1; 
	if(USART_GetITStatus(USART1,USART_IT_RXNE)!= RESET)   // ���жϵı�־
	{
		rCh = USART_ReceiveData(USART1);                  // ÿ�ν���һ���ֽڵ����ݣ�8�����أ�
		RecBuf[rCnt] = rCh;
		if(rCnt == 0)								      // ��1���ֽ��ж��Ƿ�0x66  RecBuf[0] 
		{
			rCnt = (0x66 != rCh)?0:rCnt+1;				  // �ǵĻ����������ж�
		}
		else if(rCnt == 1)
		{
			rCnt = (0xcc != rCh)?0:rCnt+1;             	 // ��2���ֽ��ж��Ƿ�0xcc   RecBuf[1] 
		}
		else if(rCnt == 2)
		{
			rCnt = (0x00 != rCh)?0:rCnt+1;               // �����ȷ�����2���ֽڵĴ�С������1���ֽ��ò���  RecBuf[2] 
		} 
		else if(rCnt == 3)
		{
			rCh1 == RecBuf[3];
			rCnt = (rCh1 != rCh)?0:rCnt+1;               // ��2���ֽڱ�ʾ��ʵ�İ�����  RecBuf[3] 
		}
		else if(rCnt == 4)
		{
			rCnt = (0x5A != rCh)?0:rCnt+1;               // �ҵ�����ͷ0x5A  RecBuf[4] 
		}
		else if(rCnt == 5)
		{	
			for(int i = 0;i < rCh1 - 2)                   // ��rCh1Ϊý������ʱ�������ݰ��ĳ��ȣ�
				rCnt++;									  // ��ȥ��ID��У��λ�����ֽڣ�
		}			
		else if(rCnt == rCh1 + 4 - 1)                     // ���Ͽ�ͷ��66cc�����ݰ�ռ���ֽ����ټ�ȥ1�����ҵ�У��λ
		{
			rCnt = 0;
			for(index=2;index<recBufLen-1;index++)        // ����У�飨��
			{
				RecCheckSum = RecCheckSum+RecBuf[index];  // ��RecBuf[2]��ʼ�ӵ�RecBuf[6]��ΪУ���	
			}
			if(RecCheckSum == rCh)                        // ���У��ɹ�����ʼ��ȡ����
			{
				g_brakingVal = RecBuf[5] & 0x7f;          // RecBuf[5]�ĵ�7λ  ����ƶ��ź�
				g_isDriverless = RecBuf[5] & 0x80;        // RecBuf[5]�ĸ�1λ  ��ż�ʻģʽ�ź�
			}
		}
	
	}
} 
		// else if(rCnt == 5)                               // ����ֱ����������2���ֽڣ��ҵ�У��λ
		//{
			//rCnt++;                                      
		//}
		//else if(rCnt == 6)
		//{
			//rCnt++;
		//}
