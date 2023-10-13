//void testPID(void)
//{
//if (j<200)
//	{
//		PIDmotor(88,88,88);
//		control(pwm1,pwm2,pwm3,0,0,0);			
//		result1[j] = w1;
//		result2[j] = w2;
//		result3[j] = w3;	
//	}
//else if (j<400)
//	{
//		PIDmotor(40,40,40);
//		control(pwm1,pwm2,pwm3,0,0,0);			
//		result1[j] = w1;
//		result2[j] = w2;
//		result3[j] = w3;
//	}
//else Stop();
//}
//void datamotor(void)
//{
//			if (j<180)
//				{
//					pwmt=j/20+1;
//			control(pwmt*100,pwmt*100,pwmt*100,0,0,0);
//				result1[j] = w1;
//				result2[j] = w2;
//				result3[j] = w3;
//					result4[j]=pwmt;
//				}
//			else if (j<360)
//				{
//					pwmt=(j-180)/20+1;
//			control(pwmt*100,pwmt*100,pwmt*100,0,0,0);
//				result1[j] = w1;
//				result2[j] = w2;
//				result3[j] = w3;
//					result4[j]=pwmt;
//					}
//			else Stop();
//}
//void testtt(void)
//{
//			control(pwmt,pwmt,pwmt,0,0,0);
//			if (pwmt<=100)
//				{
//			if (j%40>=10 && j%40<30) {
//				sum1=sum1+w1;
//				sum2=sum2+w2;
//				sum3=sum3+w3;
//				}
//			if (j%40==39) {
//				result1[pwmt] =sum1/20;
//				result2[pwmt] =sum2/20;
//				result3[pwmt] = sum3/20;
//				sum1=0; sum2=0; sum3=0;
//				}
//			}
//			if (pwmt>100) Stop();
//			j++;
//			}
//void testmotor(void)
//{
//	control(500,500,500,0,0,0);
//	dir1=0;dir2=1;dir3=0;
//}

//void testkeepingball(void)
//{
//			if (count<10)
//			{
//			ballkeeping(2); // 2 - len , 1- xuong
//			count++;
//			}
//			else 
//			{
//				ballkeeping(3);
//			}
//}
