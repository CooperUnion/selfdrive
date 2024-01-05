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

[pre-commit hook]: https://github.com/CooperUnion/selfdrive/blob/dev/.pre-commit-config.yaml
[`git blame`]: https://git-scm.com/docs/git-blame
[`git log`]: https://git-scm.com/docs/git-log
[`mdbook`]: https://rust-lang.github.io/mdBook/
[`mdformat`]: https://github.com/executablebooks/mdformat
[`selfdrive`]: https://github.com/CooperUnion/selfdrive
