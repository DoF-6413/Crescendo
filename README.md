# 0-To-Swerve
FRC Team 6413 Degrees of Freedom's Swerve Code from Scratch (No WPILIB Libraries) Each Branch is a Different Programming Member's Code

## Code Ediquiete

  Comments on EVERYTHING (Commands, Constants, etc)
  
  Organize things PROPERLY in corresponding Folders (Create Folders if Necessary)
  
  Create Issues for EVERY BRANCH
  - Every Branch Should have an Issue Linked to it
    
  LOGOUT When Finished
  - Use Credential Manager to Log Out of Github through Windows
  - Log Out of Slack
  - Log Out of Google
  - Log Out of Github

   organization of the code etiquette
 - first number = the repository
 - second number = pushes to Dev
 - third number = Issues #
 - fourth = # of commits
 - fifth = functionality (1 is working ,0 is work but not as intented, -1 is crashes ,-2 is doesn't build)

for exemple the first repository that has 2 pushes to Dev , with a issue number of 7 with  18 commits and it works but not as intented should look like    "1.2.7.18.0"

## Branch Organization

main Branch: 

- Use for Competitions
- Only Receives Pushes from Dev
- Do *NOT* Make Branches off of main

dev Branch:

-  Use for Full Systems Tests
- Only Push to Main after Tested and Clean
	- No hardcoded values
	-  Explanatory Comments
	- Code is Fully Functioning
-  Make Feature Branches off of *this* Branch

feat#[Issue#]-[name]: 

- Used for new Features on the Robot
- Branch Name is all Lowercase
- Make Feature Branches off of the Dev Branch
-  Use for Code changes
	- Changes can be as small or big as necessary
	- Only Contains what the Branch is named
		- EX. No climber code in a feat-shooter branch
-  Push to Dev when code is Tested and Clean
	- No hardcoded values
	-  Explanatory Comments
	- Code is Fully Functioning

bugfix#[Issue#]-[name]: 

- Used for a bugfix of a feature from Dev
- Branch Name is all Lowercase
- Make BugFix Branches off of the Dev Branch

chore#[Issue#]-[name]: 

- Used for cleaning code
- Branch Name is all Lowercase
- Make Chore Branches off of the Dev Branch
- Mainly used by mentor or lead


## Useful Git Bash Commands
- git add .
	- stages all code to prepare for commit and push
- git commit -m "[Insert Message Here]" 
	- Saves code locally with a msg of what was done to the code
- git push
	- Pushes Code to the Cloud
- git fetch
	- tells laptop their are new changes
- git pull
	- puts new changes on laptop 
	- git pull does git fetch
- git pull origin Dev
	- pulls changes from Dev
- git checkout [branch name]
	- Changes your branch
- git checkout -b [branchname]
	- Creates new branch off of current branch
- git branch
	- Shows all branches that have been pulled on laptop
- git branch --all
	- Shows all branches since last fetch
- git config user.email [User Email]
  - Configures Email for THAT specfic Folder
  - DO NOT USE GLOBAL ON SHARED LAPTOPS
  - Used to Properly Lable who Wrote what Code through Commits (Not Access or PRs)
- git config user.name [Github Username]
  - Configures Username for THAT Specific Folder
  - DO NOT USE GLOBAL ON SHARED LAPTOPS
  - Used to Properly Lable who Wrote what Code through Commits (Not Access or PRs)
