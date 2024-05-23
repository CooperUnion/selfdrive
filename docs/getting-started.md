# Getting Started

[`selfdrive`] is a rather *complicated* repo.

If you've never worked on a project of this scale, it can be daunting to
know where to start. The goal of this guide is to give you enough of a
foothold to start asking the right questions and to begin your adventure
here at Autonomy Lab.

## Fundamentals

Before we even get to cloning [`selfdrive`], you need to have the basic
mechanical skills required to traverse a shell. [MIT's Missing Semester]
serves as a great starting point for just this. The following table
lists the lectures you should watch if you plan to work on a specific
Autonomy Lab group:

| Lecture                       | Hardware | Firmware | Algorithms |
| ----------------------------- | -------- | -------- | ---------- |
| [Course overview + the shell] | ✓\*      | ✓\*      | ✓\*        |
| [Shell Tools and Scripting]   | ✓        | ✓\*      | ✓\*        |
| [Editors (Vim)]               |          |          |            |
| [Data Wrangling]              |          | ✓        | ✓\*        |
| [Command-line Environment]    | ✓        | ✓\*      | ✓\*        |
| [Version Control (Git)]       | ✓\*      | ✓\*      | ✓\*        |
| [Debugging and Profiling]     |          | ✓        | ✓          |
| [Metaprogramming]             |          | ✓        | ✓          |
| [Security and Cryptography]   |          |          |            |
| [Potpourri]                   |          | ✓        | ✓          |
| [Q&A]                         |          | ✓        | ✓          |

> *Note: an `*` denotes you should also do the associated exercises.*

At a minimum, to clone [`selfdrive`], you should watch and do the
exercises for [Course overview + the shell] and [Version Control (Git)],
as these lectures provide you the fundamentals to hit the ground
running.

## System Configuration

Since [`selfdrive`] consists of software written in different
programming languages with a wide variety of dependencies, we've made an
effort to make getting your machine ready as painless as possible with
the [Nix package manager] and [direnv]. You can find cloning
instructions and system setup information on the repo's [`README.md`].

## Repo Layout

> Currently, [`selfdrive`] does not respect this structure. We're in the
> process of migrating over to this layout. Sorry for the time being.

```
selfdrive/
+-- can/
|   +-- network.yml
|
+-- docs/
|
+-- components/
|   +-- component/
|   |   +-- docs/
|   |   +-- src/
|   |   +-- component.toml
|   |
|   +-- ...
|
+-- lib/
|
+-- projects/
|   +-- project/
|   |   +-- docs/
|   |   +-- project.toml
|   |
|   +-- ...
|
+-- ...
```

There's quite a bit to unpack here, so let's get to it!

### [`components/`]

The [`components/`] directory contains components that serve as the
building blocks of a project.

Each component has a `docs/` directory that contains [symlinks] to
relevant documentation. These symlinks need to stay up to date as they
will automatically get triggered for manual review on pull requests to
ensure that documentation stays up to date. Having these symlinks also
ensures that anyone who wishes to get up to speed on a component has a
quick list of relevant documentation to read.

The `src` directory contains the source files. The contents and layout
of this directory highly depend on the component language(s) and use
case. It is best to see existing components to get a feel for it.

Finally, `component.toml` contains metadata related to the component.

### [`can/`]

[`can/`] describes the CAN networks of our projects. Here, you'll find
definitions for networks, messages, and signals for [OpenCAN] to
process.

### [`docs/`]

The [contributing guidelines] cover the structure for documentation. You
may notice that it closely follows the structure of [`selfdrive`]
itself.

### [`lib/`]

The [`lib/`] directory contains shared libraries used in components.
Since these are generalized pieces of software, the contents of each
directory can wildly vary. Feel free to explore these directories if
you're curious.

### [`projects/`]

The [`projects/`] directory contains projects that are composed of
various components. Like a component, each project has a `docs/`
directory with symlinks to relevant documentation and a `project.toml`
with related metadata.

### What About Everything Else?

It'd be impractical to cover every last file and directory in
[`selfdrive`]. Unfortunately, there's no shortcut to knowing where you
can find everything or how various bits of code interact, but there are
tools that can help!

- [`fd`]: a simple, fast and user-friendly alternative to `find`
- [`fzf`]: a command-line fuzzy finder
- [`rg`]: ripgrep recursively searches directories for a regex pattern

## So What's Next?

Now that you have a general idea of how [`selfdrive`] works, it's time
to move on to group-specific getting started guides. Feel free to move
along to these without finishing all of the suggested [missing semester]
lectures; however, understand they do nothing but benefit you!

[command-line environment]: https://missing.csail.mit.edu/2020/command-line/
[contributing guidelines]: contributing.md#directory-structure
[course overview + the shell]: https://missing.csail.mit.edu/2020/course-shell/
[data wrangling]: https://missing.csail.mit.edu/2020/data-wrangling/
[debugging and profiling]: https://missing.csail.mit.edu/2020/debugging-profiling/
[direnv]: https://direnv.net/
[editors (vim)]: https://missing.csail.mit.edu/2020/editors/
[metaprogramming]: https://missing.csail.mit.edu/2020/metaprogramming/
[missing semester]: https://missing.csail.mit.edu/
[mit's missing semester]: https://missing.csail.mit.edu/
[nix package manager]: https://nixos.org/
[opencan]: https://github.com/opencan
[potpourri]: https://missing.csail.mit.edu/2020/potpourri/
[q&a]: https://missing.csail.mit.edu/2020/qa/
[security and cryptography]: https://missing.csail.mit.edu/2020/security/
[shell tools and scripting]: https://missing.csail.mit.edu/2020/shell-tools/
[symlinks]: https://en.wikipedia.org/wiki/Symbolic_link
[version control (git)]: https://missing.csail.mit.edu/2020/version-control/
[`can/`]: https://github.com/CooperUnion/selfdrive/tree/dev/can
[`components/`]: https://github.com/CooperUnion/selfdrive/tree/dev/components
[`docs/`]: https://github.com/CooperUnion/selfdrive/tree/dev/docs
[`fd`]: https://github.com/sharkdp/fd
[`fzf`]: https://github.com/junegunn/fzf
[`lib/`]: https://github.com/CooperUnion/selfdrive/tree/dev/lib
[`projects/`]: https://github.com/CooperUnion/selfdrive/tree/dev/projects
[`readme.md`]: https://github.com/CooperUnion/selfdrive/blob/dev/README.md
[`rg`]: https://github.com/BurntSushi/ripgrep
[`selfdrive`]: https://github.com/CooperUnion/selfdrive
