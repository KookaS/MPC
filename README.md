# MPC
EPFL project | Model Predictive Control

### Code

The programm runs differently for the deliverables, make sure to uncomment the correct part in src/MPC_Control_z.m

Execute then the deliverable to see the simulation.

### Generate SSH
    ssh-keygen

    ssh-add

    cat $HOME\.ssh\id_rsa.pub
copy stuff and paste it in Github -> Settings -> SSH and GPG keys -> New

### Init
in the folder you want to work on:

    git clone git@github.com:KookaS/MPC.git

### Branches
You need everytime you code that you are in an other branch than the main one, especially if you commit and push your code.

I would suggest that before creating a new branch you pull the recent changes:

    git checkout master

    git pull
To create a new branch from your repo:

    git checkout -b <your-branch-name>
Pushing code
When you are in your branch, check first if there are conflicts with main branch:

Pycharm:

click on the icon top right update project
Terminal:

    git checkout master

    git pull

    git checkout <your-branch-name>

git merge master
Now that your are sure that the code still runs well you need to commit and push your code:

Pycharm:

click the icon on the top right after reboot of Pycharm: commit

    enter your commit message, explain what you did

click the icon push and push your branch to github
if you don't see your branch pushed and it is your first attempt click again on push:

you'll see <your-branch-name> -> master
click on master and define what it is(origin/master) with the url of the project. Push again and then link Pycharm to Github

Terminal:

    git commit -m "your message to explain here"

git push .....      you will have some comments like push set-upstream, just do what they say
Now you need to merge your branch on github. YOu go to the page of the project and click create PR of you branch to the main one.


