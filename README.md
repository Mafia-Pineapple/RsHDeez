# RsHDeez
Rs1 Repository 

## Setting up SSH
1. Open terminal
2. Paste the following into terminal and change the email: `ssh-keygen -t ed25519 -C "your_email@example.com"`
3. When you're prompted to "Enter a file in which to save the key", you can press Enter to accept the default file location
4. Repeat for the passphrase
5. Add your SSH key to the ssh-agent with: `eval "$(ssh-agent -s)"`
6. Run: `cat ~/.ssh/id_ed25519.pub`
7. Then select and copy the contents of the id_ed25519.pub file displayed in the terminal to your clipboard
8. Go to your github account -> settings -> SSH and GPG keys -> New SSH key
9. Give the key a name and paste the contents of the id_ed25519.pub file in the body
10. Click Add SSH key

## Setting up and Cloning Git
1. Open terminal
2. Run: `cd 41068_ws/src`
3. To clone the github run: `git clone git@github.com:Mafia-Pineapple/RsHDeez.git`
4. At the end of this process you also need to inform your computer or your git identity and set a preference for rebasing by executing the three commands in a terminal.

  `git config --global user.name "<your_name>"`

  `git config --global user.email "<your_email>"`

  `git config --global pull.rebase false`

## Creating a Package
1. Open terminal
2. Run: `cd 41068_ws/src/RsHDeez`
3. Then run: `ros2 pkg create --build-type ament_cmake <package_name>`, making sure to give the package a unique name
4. A package with a CMakeLists.txt and package.xml file will generate
