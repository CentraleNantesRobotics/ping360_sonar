# Contributing to this package

👍🎉 First off, thanks for taking the time to contribute! 🎉👍

We're really glad you're reading this, because we need volunteer developers to help improve this project.

First, here are some resources:

  * [ROS](http://wiki.ros.org/)
  * [Ping360 Documentation](https://docs.bluerobotics.com/ping-protocol/pingmessage-ping360/)
  * [ping-protocol Python Lib](https://pypi.org/project/bluerobotics-ping/)

## Testing

TODO

## Linting

You may notice that a CI job is triggered after every new push, these CIs assure that the code format follows the PEP8 standard imposed by the ROS team.
To make sure your changes are well formatted, we supplied a pre-commit config file that can be used to verify code formatting before commit.

### Installing "pre-commit"

pre-commit is written in python and can be easily installed using : ```pip install pre-commit```

### Configuring "pre-commit"

In your local repo folder, execute ```pre-commit install``` to install git hooks in your .git/ directory.

### Using "pre-commit"

Once you're ready to commit your new changes, run :
```
$ git add .
$ git commit -m "Commit message CHANGE ME"
```
This will trigger the pre-commit hook and output the following:
```
autopep8.................................................................Passed/Failed
Flake8...................................................................Passed/Failed
[develop f726bfe] Add the minAngle parameter + readme update
 3 files changed, 51 insertions(+), 17 deletions(-)
```

If everything goes well, your changes will be commited. If there's an issue autopep8 will try to fix it (don't forget to stage the new edits), so just run the commit command a second time.
I also recommend installing autopep8 using pip and using it as a formatter in you're IDE. (VS Code has that by default).


For more info about using precommits, please refer to : https://ljvmiranda921.github.io/notebook/2018/06/21/precommits-using-black-and-flake8/
## Submitting changes

Please send a [GitHub Pull Request](https://github.com/CentraleNantesRobotics/ping360_sonar_python/pull/new/develop) with a clear list of what you've done (read more about [pull requests](http://help.github.com/pull-requests/)). When you send a pull request, we will love you forever if you update the README or add tests too. We can always use more tests. Please follow our coding conventions (below) and make sure all of your commits are atomic (one feature per commit).

Always write a clear log message for your commits. One-line messages are fine for small changes, but bigger changes should look like this:

    $ git commit -m "A brief summary of the commit
    > 
    > A paragraph describing what changed and its impact."

## Coding conventions

Start reading our code and you'll get the hang of it. We optimize for readability:

  * We indent using two spaces (soft tabs)
  * We ALWAYS put spaces after list items and method parameters (`[1, 2, 3]`, not `[1,2,3]`), around operators (`x += 1`, not `x+=1`), and around hash arrows.
  * This is open source software. Consider the people who will read your code, and make it look nice for them. It's sort of like driving a car: Perhaps you love doing donuts when you're alone, but with passengers the goal is to make the ride as smooth as possible.

Thanks,
Anas @ Centrale Nantes Robotics
