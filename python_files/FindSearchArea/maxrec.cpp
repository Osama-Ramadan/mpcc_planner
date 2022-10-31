#include<bits/stdc++.h>
using namespace std;

char w[20][20];
int tl[20][20] = {0}, tr[20][20] = {0}, h[20][20] = {0}, l[20][20]={0}, r[20][20]={0};
int ans, R,C, xr,xc, ar1, ac1, ar2, ac2; 

int largestRect()
{
    int area = 0;
    
    for(int i=0; i<20;i++) for(int j=0; j<20;j++) l[i][j] = r[i][j] = 1<<28;
    
   	for(int i=1; i<=R;i++){
   	    for(int j=1; j<=C;j++)
   		    if(w[i][j]!='#') tl[i][j] = tl[i][j-1]+1;
   			else tl[i][j] = 0;
 
   		for(int j=C; j>=1; j--)
   			if(w[i][j]!='#') tr[i][j] = tr[i][j+1]+1;
   			else tr[i][j] = 0;
   		
   		for(int j=1; j<=C; j++)
   			if(w[i][j]!='#') h[i][j] = h[i-1][j]+1;
   			else h[i][j] = 0;
   		
   		for(int j=1; j<=C; j++)
   			if(w[i][j] != '#') l[i][j] = min(tl[i][j], l[i-1][j]);
   		
   		for(int j=1; j<=C; j++)
   			if(w[i][j] != '#') r[i][j] = min(tr[i][j], r[i-1][j]);
   	
   		
   		for(int j=1; j<=C;j++){
   			int offset = 0;
   			if((r[i][j]+l[i][j]-1)*h[i][j] > 0){
   			    if(xr >= i-h[i][j]+1 && xr <= i && xc >= j-l[i][j]+1 && xc <= j+r[i][j]-1) offset = 1<<28;
   				printf("Top left = (%d,%d)  Bottom right = (%d,%d)\n", i-h[i][j]+1, j-l[i][j]+1,i, j+r[i][j]-1);
   				printf("Area = %d\n", (r[i][j]+l[i][j]-1)*h[i][j]);
   				printf("Included X? %d\n", xr >= i-h[i][j]+1 && xr <= i && xc >= j-l[i][j]+1 && xc <= j+r[i][j]-1 );
   				printf("Area with offset = %d\n\n", (r[i][j]+l[i][j]-1)*h[i][j] + offset);
   					
   				if(area <= (r[i][j]+l[i][j]-1)*h[i][j] + offset){
   					area = max(area, (r[i][j]+l[i][j]-1)*h[i][j] + offset);
   						ar1 = i-h[i][j]+1; ac1 = j-l[i][j]+1; ar2 = i; ac2 = j+r[i][j]-1;
   				}
   			}
   		}
   	}
   
    return area;
}

int main(){
	scanf("%d %d", &R, &C);
	
	for(int i=0; i<20;i++) for(int j=0; j<20;j++) w[i][j] = '#';
	
	for(int i=1; i<=R; i++){
		getchar();
		for(int j=1; j<=C; j++){
			w[i][j] = getchar();	
			if(w[i][j] == 'X'){ xr = i; xc = j;}
		} 
	}
	
	ans = largestRect() - (1<<28);

	printf("Largest Rect Containing X is (%d,%d) to (%d,%d), with area = %d\n", ar1, ac1, ar2, ac2, ans);
	return 0;	
}