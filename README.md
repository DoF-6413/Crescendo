# Crescendo
Code Base for FRC Team 6413's 2024 Crescendo Robot Code

## Code Etiquette

  Comments on EVERYTHING (Commands, Constants, etc)
  
  Organize things PROPERLY in corresponding Folders (Create Folders if Necessary)
  
  Create Issues for EVERY BRANCH
  - Every Branch Should have an Issue Linked to it
    
  LOGOUT When Finished
  - Use Credential Manager to Log Out of Github through Windows
  - Log Out of Slack
  - Log Out of Google
  - Log Out of Github

   Beta Numbers
 - first number = the repository
 - second number = pushes to Dev
 - third number = issue #
 - fourth number = # of commits
 - fifth number = functionality (1 is working, 0 works but not as intented, -1 is crashes, -2 is doesn't build)

For Example, the First Repository that has 2 Pushes to Dev, on the branch with an issue number of 7 with 18 commits, which it works but not as intented, should look like: "1.2.7.18.0"

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
- Branch name is all lowercase
- Make bugfix Branches off of the Dev Branch

chore#[Issue#]-[name]: 

- Used for cleaning code
- Branch name is all Lowercase
- Make chore branches off of the Dev Branch
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
  - Used to properly label who wrote what code through Commits (Not Access or PRs)
- git config user.name [Github Username]
  - Configures Username for THAT Specific Folder
  - DO NOT USE GLOBAL ON SHARED LAPTOPS
  - Used to properly label who wrote what code through Commits (Not Access or PRs)
