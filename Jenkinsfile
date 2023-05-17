pipeline {
    agent any
    options {
        skipStagesAfterUnstable()
    }
    stages {
        stage('Build') { 
            steps { 
                script{
                    img = docker.build("nu-ideas-lab/grex_dev", "--platform linux/amd64 .")
                }
            }
        }
        stage('Test'){
            steps {
                 echo 'Empty'
            }
        }
        stage('Deploy') {
            environment {
                tag_safe = env.BRANCH_NAME.replaceAll('[^a-zA-Z0-9]','_')
            }
            steps {
                script{
                        docker.withRegistry('https://ghcr.io', 'ideas-ci-github') {
                        img.push("${env.tag_safe}")
                        if (env.BRANCH_NAME == 'master') {
                            img.push('latest')
                        }
                    }
                }
            }
        }
    }
}