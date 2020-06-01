import * as data from './data.js';

class Person {
    constructor(name, index) {
        this.name = name;
        this.index = index;
        this.bias = 0;
        this.factor1 = 0;
    }
};

class Movie {
    constructor(name, ratings) {
        this.name = name;
        this.bias = 0;
        this.factor1 = 0.1;
        this.ratings = ratings;
    }
}
class Rating {
    constructor(actualValue, person) {
        this.actualValue = parseFloat(actualValue);
        this.person = person;
        this.prediction = 0;
        this.penalty = 0;
        this.displayedValue = actualValue;
    }
    set nice(value) {
        this.prediction = (1 / (1 + Math.exp(-value))).toFixed(3);
        if (this.actualValue >= 0) {
            this.penalty = (this.prediction - this.actualValue) ** 2;
            this.displayedValue = `pred: ${this.prediction}, act: ${this.actualValue}`;
        } else {
            this.displayedValue = `pred: ${this.prediction}`;
        }
        
    }
}

var app = new Vue({
    el: '#app',
    data: {
      persons: [],
      movies: [],
      penalty: 0,
      display: 'actual',
    },
    computed: {
      penaltyFormatted: function( ){
          return `całkowity błąd  ${this.penalty} (im mniej tym lepiej)`;
      }
    },
    watch: {
        persons: {
            handler: function(value, old_value) {
                this.updatePredictions();
            },
            deep: true
        },
        movies: {
            handler: function(value, old_value) {
                this.updatePredictions();
            },
            deep: true
        },
    },
    mounted: function () {
        for(let i = 0; i < data.data[0].ratings.length; i++) {
            this.persons.push(new Person(`P${i}`, i));
        }
        for(const m of data.data) {
            let ratings = Array(m.ratings.length);
            for(let i = 0; i < m.ratings.length; i++) {
                ratings[i] = new Rating((m.ratings[i] / 5 - 0.1).toFixed(3), this.persons[i]);
            }
            this.movies.push(new Movie(m.name, ratings));
        }
    },
    methods: {
        updatePredictions: function() {
            const lambda = 0.1;
            let penalty = 0;
            for(let personIndex = 0; personIndex < this.persons.length; personIndex++) {
                let person = this.persons[personIndex];
                penalty += lambda * (person.bias ** 2 + person.factor1 **2);
                for(let movieIndex = 0; movieIndex < this.movies.length; movieIndex++) {
                    let movie = this.movies[movieIndex];
                    if (personIndex == 0)
                        penalty += lambda * (movie.bias ** 2 + movie.factor1 **2);
                    movie.ratings[personIndex].nice = parseFloat(movie.bias) + parseFloat(person.bias)
                    + parseFloat(movie.factor1) * parseFloat(person.factor1);
                    penalty += movie.ratings[personIndex].penalty;
                }
            }
            this.penalty = penalty;
        },
        optimizeStep: function() {
            let penalty = this.penalty;
            let deltaBias = (Math.floor(Math.random() * 3) - 1) / 100;
            let deltaF1 = (Math.floor(Math.random() * 3) - 1) / 100;
            let variable = undefined;
            if (Math.random() < 0.5) {
                let personIndex = Math.floor(Math.random() * this.persons.length);
                variable = this.persons[personIndex];
            } else {
                let movieIndex = Math.floor(Math.random() * this.movies.length);
                variable = this.movies[movieIndex];
            }
            variable.bias = parseFloat(variable.bias) + deltaBias;
            variable.factor1 = parseFloat(variable.factor1) + deltaF1;
            this.updatePredictions();
            if (this.penalty > penalty) {
                variable.bias = parseFloat(variable.bias) - deltaBias;
                variable.factor1 = parseFloat(variable.factor1) - deltaF1;
            }
        },
        optimize: function() {
            for(let i = 0;  i < 1000; i++)
                this.optimizeStep();
        }
    }
  })
