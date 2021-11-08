# IGVC selfdrive monorepo

Root directory structure:

```
selfdrive/
          can/    - CAN definitions and tools
          common/ - Common libraries 
          dbw/    - Drive-by-wire firmware, watchdog, UI, etc
          ros/    - ROS environment, packages
          tools/  - Miscellaneous tools and resources
```

------------------------------------------------------------------------------

## Workflow

To keep a clean history and functional repo, here are the steps to follow:

1. Create an issue for what you are trying to address.
2. Create a merge request (which will create a branch for you). Make sure
   that the issue number (e.g. #1) is in the title.
3. Once done with the changes, get someone to review your code.
4. Merge to master; squash unless you have a reason not to.

## Things to note

- Add CI and tests for code wherever possible.
- Don't publish the code in this repo; we will be making strategically timed
  source releases. This is to protect our competitive advantage before the
  competition and to protect any development signing keys we may keep here.
