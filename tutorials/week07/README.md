# Week 7 

<img src="https://miro.medium.com/max/1200/1*bZHBo75FSyKre5pk2-HPmw.png"  style="zoom: 30%;"/>

While we have wrapped our head around the idea of a mutex (in providing exclusive access), this has resulted in threads waiting for access and not having much control over which threads will run. We have effectively seen some data races, threads being starved and hogging data, having to resort to sleeping threads to provide access. 

Enter the concept of conditional variable *(aka convar)* that lets us inform other threads it's their turn to process data. And we can unchain our poor threads.

Have a look at the example here [conditional variable](https://en.cppreference.com/w/cpp/thread/condition_variable), the worker thread waits until it is notified to progress. This allows us to have threads only operate when we provide them with new data, and allows us to do things in parallel.

**[Prework is Quiz3](../../quizzes/quiz3/README.md)**

**[Tutorial Questions](./TUTORIAL.md)**







