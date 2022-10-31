// So goal top = 275 , 12.5
// Goal bottom = 270, 360
// set goals for set up
//if input 0 set goal to low y value ("top")
void SetGoals(int top){
if (top==0){
    goal[0]= 275;
    goal[1]= 12.5;
    goal2[0]= 270;
    goal2[1]= 360;
}
else{
    goal2[0]= 275;
    goal2[1]= 12.5;
    goal[0]= 270;
    goal[1]= 360;
}
}