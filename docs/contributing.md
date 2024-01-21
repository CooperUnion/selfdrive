# Contributing

Thank you for taking time out of your day to contribute! We're excited
to have you contribute to our documentation, but before you do, let's go
over how best to contribute.

## Before You Start

Before you get to writing anything, ask yourself a few questions:

- Can existing documentation be improved?
- If the existing documentation is insufficient, can it be expanded?
- Have you generalized to a reasonable degree?
- Is what you're about to write correct to the best of your knowledge?

> *As an aside, no documentation is better than wrong documentation!*

## Writing Style

*Active voice over passive voice where possible.*

Passive voice is less concise than the active voice, making
documentation difficult to read. More importantly, passive voice can
hide the agent of a sentence, diminishing your contribution to the work
you put so many hours into!

*Talk to the reader.*

Remember, we read documentation far more often than we write it. That
said, write documentation that's easy to read; this means talking to the
reader to engage them with the material. We've all read dry
documentation before; let's avoid it here at Autonomy Lab.

*You're writing on behalf of Autonomy Lab.*

Since we're a lab, avoid the word I. Work done for the lab, although
individual at times, can become collaborative in an instant. There's no
need to worry about your contribution getting lost in the chaos as it'll
always be traceable with [`git blame`] and [`git log`].

*Be professional, but don't be afraid to have some fun!*

[`selfdrive`] is a public repo: any contribution you make gets tied to
your name forever on the internet, so be professional! That said, don't
be afraid to have some fun! Humor is always welcome where appropriate to
give our work some life!

## Formatting

We use [`mdbook`] to render our documentation. If this is your first
time using [`mdbook`], please read over its documentation to get a feel
for how to navigate it.

We've also decided to normalize our markdown using [`mdformat`]. If your
markdown isn't normalized, it will trigger our [pre-commit hook]. To
normalize a markdown file, it suffices to run the following in the root
of [`selfdrive`]:

```
$ mdformat path/to/file.md
```

Each markdown file should, at the very least, look like this:

```md
# Title

Introductory blurb...

## First Topic

...
```

## Directory Structure

Our documentation has the following directory structure:

```
docs/
+-- components/
|   +-- complicated-component/
|   |   +-- README.md
|   |   |
|   |   +-- ...
|   +-- component.md
|
+-- concepts/
|   +-- complicated-concept/
|   |   +-- README.md
|   |   |
|   |   +-- ...
|   +-- concept.md
|
+-- projects/
|   +-- project/
|       +-- complicated-topic/
|       |   +-- README.md
|       |   |
|       |   +-- ...
|       +-- topic.d/
|       |   +-- figure.png
|       +-- topic.md
|
+-- README.md
+-- SUMMARY.md
|
+-- ...
```

Typically, a new addition to our documentation is a single markdown
file. If we split the documentation into multiple files, we can
represent it as a directory with a `README.md`, which serves as the
introductory point.

All file paths should be lowercase (except for `README.md`), and use
dashes for spaces.

If you need to include a file (i.e. a figure) but are only using a
single markdown file, a folder with the same base name but suffix `.d`
can be created (i.e. `topic.md -> topic.d`). Otherwise, included files
specific to the documentation should reside in the same folder.

## Figures

When creating figures, make them in such a manner that they're editable
at a later time. The exception to this are images. When adding images,
make sure they don't contain any personally identifiable EXIF data (i.e.
the location of your house) before adding it to [`selfdrive`]!

When adding figures ***not*** created by Autonomy Lab, they must be
appropriately accredited. Only link to such figures, and do not add them
to the [`selfdrive`] repo. External figures must also be compatible with
the [CC BY-NC-SA 4.0].

> Additionally, if the figure you add is a binary file track it with
> [Git LFS].

## When in Doubt, Look at Existing Work

Not sure how to do something? Check if there's already existing work
that achieves a similar pattern. If you can't find anything that quite
matches what you're looking to write, take your own stab at it. The nice
part of having PRs is that the Lab can give you feedback on your work so
that we can all converge on an ideal solution.

[cc by-nc-sa 4.0]: https://creativecommons.org/licenses/by-nc-sa/4.0/legalcode
[git lfs]: https://git-lfs.com/
[pre-commit hook]: https://github.com/CooperUnion/selfdrive/blob/dev/.pre-commit-config.yaml
[`git blame`]: https://git-scm.com/docs/git-blame
[`git log`]: https://git-scm.com/docs/git-log
[`mdbook`]: https://rust-lang.github.io/mdBook/
[`mdformat`]: https://github.com/executablebooks/mdformat
[`selfdrive`]: https://github.com/CooperUnion/selfdrive
