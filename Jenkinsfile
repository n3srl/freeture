pipeline {
    agent  {
        docker { image 'freeture-dev' }
    }

    stages {
        stage('Build') 
		{
            steps {
                echo 'Building..'
            }
        }
        stage('Test') 
		{
            steps {
                echo 'Testing..'
            }
        }
    }
}