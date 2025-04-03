GIT Help (Basic use guide)
===================================
Git is a version control system for tracking changes in computer files and coordinating work on those files among multiple people. It is primarily used for source code management in software development. It is the de-facto version control system of open source development, our git repositories are hosted on github, providing you with **individual private remote** repositories for the subject. 

[A video guide to be used in conjunction with this information is available on canvas](https://canvas.uts.edu.au/courses/26214/pages/github-and-using-git?module_item_id=1199454)

To use git, you will need to use the Linux command line. Instructions below assume you have a terminal open. Terminal commands will be highlighted `like this`. If you see any notation using angle brackets `<like-this>`, you will need to substitute it with a variable related to you. For instance `<username>` requires your **git username**, (maybe Johno if that is your git username ) don't use the `<>` we simply use this indicate that you have to complete a substitution.

Once you have accepted the classroom invite with your git account, a new git repository will be created called  `pfms-2023a-<username>` which contains a clone of the subject material. Github provides a web interface where you can manage your git repository at: `https://github.com/41012/pfms-2023a-<username>`. At any point in time your web interface and your local repository could be different, read on to understand why and how to manage this.

Setting up the repository on your workstation
------------------------------------
This is a one-time process.

I would strongly advocate to store your ssh key on GitHub, this allows you to remove the need to enter a password each time you perform any operations on your Ubuntu system, it has 3 steps that needs to be undertaken only once:

1. [Generate a new SSH key](https://docs.github.com/en/github/authenticating-to-github/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent#generating-a-new-ssh-key)
2. [Adding your SSH key to the ssh-agent](https://docs.github.com/en/github/authenticating-to-github/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent#adding-your-ssh-key-to-the-ssh-agent)

3. [Add new SSH key to yout GitHub Account](https://docs.github.com/en/github/authenticating-to-github/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account)


At your desired location in your file system clone the repository (mine is under a subfolder `~/git` where I keep all my git repositories). If you click on the **Code** button (which is Green) on your repository website and then ** Select SSH instead of HTTPS in the Tab*** you will find the complete URL for cloning (it begins with git@github and has .git added to your webpage).
```
git clone git@github.com:41012/pfms-2023a-<username>.git
```
After executing this, you will find that a folder pfms-2023a-<username> is created, this is your local repository. It has your code, with all files you have pushed to date that are specific to you.

Unless you have used git previously on your device you will need to let Git know your identity
```
git config --global user.name "<your_name>"
git config --global user.email "<your_email>"
```
For instance ``git config --global user.name "John Smith"`` and ``git config --global user.email "John.Smith@email.com"``

Obtaining tutorial material
------------------------------------
We will make an announcement on [TEAMS] as well as Canvas when material is available. On each instance you will need to execute the following within any folder of your local repository.
```
git fetch origin subject
git merge origin/subject --allow-unrelated-histories
git push
```
The above will work seamlessly, as long as you have not created files/folders with the same name as those we are distributing. 
The merging happens on the level of your local repository and last comment `git push` pushes the merged results back to the remote repository, so the result is able to be seen on git.
If the merge responds with errors, drop a [TEAMS] message (under software channel.)


Obtaining quiz material and feedback
------------------------------------
We will make an announcement on [TEAMS] as well as Canvas when material is available. On each instance you will need to execute the following within any folder of your local repository.
```
git pull
```
**I would strongly suggest to ``git pull`` whenever starting work again on your local repository anyway**

Managing your own files
------------------------------------
In order to examine the status of your local changes use:
```
git status 
```
If you wish to stage files for commit:
```
git add <file_or_folder> 
```
To commit files to your local repo, and specify a `<message>` which describes the nature of the changes you have made use:
```
git commit -m “<message>”
```
It is important to use descriptive commit messages so that later you can make sense of your commits using `git log`.

**NOTE:** If you do not specify a message, you will be prompted for one, a terminal-based text editor will appear to write a message, after writing it hit CTRL+X, then Y, then ENTER.

To publish your local commits to your hosted repository use:
```
git push
```

You can find **much** more info about git online. Be aware that [git](https://git-scm.com) is not the same as [github](https://github.com). git is a version control system. github is just a popular website that provides hosted git repositories.

Git Workflow Explained
------------------------------------

Git is not cloud storage (ie. DropBox/Google Drive), it is designed for managing code. Managing code well would imply that their is a version of your code which compiles / does not break other dependent code, that code is stable and available to you (and your team if working in a team). 

Obviously, cloud storage has no understanding of this requirement, and simply updates your storage with any changes, as it was designed for documents/text/images. Further, cloud storage has no knowledge of which files need to be backed up, programming has many intermediate files that are simply a by-product of build/link process and specific to your computer. 

Therefore, the onus of managing your code lies on the developer, *you decide* when to *push* the code to hosted/remote repository (from here on repository is referred to as *repo*). 

To allow users to nominate which code to store in version control, git has a staging process. You must therefore notify git which code to consider using the ``git add`` command.

To achieve the set objectives of versioning code git has a few layers, it considers a local and a remote repo, that is, git allows you to locally manage your code as well, work on it and then push it to the remote. In this way there is versioning on your local repo for code not ready to be pushed to remote repo. Therefore we have ``git commit`` to achieve version control on your local repo, and ``git push`` to push it to the remote repo. Until the push your code does not exist on remote repo (therefore is not backed-up/in-the-cloud so to speak). 

To obtain code from remote repo you use ``git pull`` which is actually a two phase process ``git fetch`` followed by a ``git merge``.

An illustration of this follows- image from https://tex.stackexchange.com/questions/70320/workflow-diagram)
![alt text](https://i.stack.imgur.com/5V7uJ.png "Git workflow")


------------------------------------



[TEAMS]: https://teams.microsoft.com/l/team/19%3aEvIKpMNFj6OfxI6fWPJ_1oHiYHAOFV63-Ce1VRLWJIE1%40thread.tacv2/conversations?groupId=218d8a36-d74d-409a-bfdd-fcd539accde1&tenantId=e8911c26-cf9f-4a9c-878e-527807be8791

